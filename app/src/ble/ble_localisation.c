#include "ble_localisation.h"

#include <zephyr.h>
#include <kernel.h>
#include <posix/time.h>
#include <sys/byteorder.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>

#include <logging/log.h>

#include "accumulator.h"
#include "location.h"
#include "ble_network.h"


LOG_MODULE_REGISTER(ble_localisation, LOG_LEVEL_INF);


#define MAX_LOCALISATION_BEACONS 16

#define SCAN_PERIOD 5000
#define MAX_MISSED_SCAN_PERIODS 5

#define BLE_LOCALISATION_STACK_SIZE 1024
#define BLE_LOCALISATION_PRIORITY 5

/* This delay is so that BLE logs can be seen from the shell */
#define BLE_LOCALISATION_STARTUP_DELAY 500


/* Scan parameters for BLE advertisement scanning */
static const struct bt_le_scan_param scan_params = {
        .type = BT_HCI_LE_SCAN_PASSIVE,
        .options = BT_LE_SCAN_OPT_NONE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window = BT_GAP_SCAN_FAST_WINDOW
};

/* List to append ibeacon data to */
static sys_slist_t ibeacon_list;

/* Work item to filter scanned ibeacons */
static struct k_work ibeacon_filter_work;

/* Mutex to protect access to ibeacon_list */
static struct k_mutex ibeacon_list_mutex;

/* Memory slab to allocate ibeacon data from */
K_MEM_SLAB_DEFINE_STATIC(ibeacon_mem, sizeof(struct ibeacon_packet), 
        MAX_LOCALISATION_BEACONS, 4);

/* Message queue to pass ibeacon packets to ibeacon filter */
K_MSGQ_DEFINE(ibeacon_msgq, sizeof(struct ibeacon_packet), 30, 4);

/* Mesage queue to request location data for ibeacon from server */
K_MSGQ_DEFINE(ibeacon_request_msgq, sizeof(struct ibeacon), 30, 4);

/**
 * @brief Empties the ibeacon_list
 * 
 * @return 0 on success, else negative error code
 */
static int empty_ibeacon_list(void)
{
    struct ibeacon_packet *beacon;
    sys_snode_t *node;

    int ret = k_mutex_lock(&ibeacon_list_mutex, K_MSEC(1));
    if (ret != 0) {
        return ret;
    }

    while ((node = sys_slist_get(&ibeacon_list)) != NULL) {
    
        beacon = SYS_SLIST_CONTAINER(node, beacon, next);

        k_mem_slab_free(&ibeacon_mem, (void **)&beacon);
    }

    k_mutex_unlock(&ibeacon_list_mutex);
    return 0;
}

/**
 * @brief Checks if an ibeacon is already in the ibeacon_list
 * 
 * @param ibeacon ibeacon to look for in list
 * @return EEXIST if the ibeacon is in the list
 *         0 if the ibeacon isn't in the list and no errors occured
 *         negative error code otherwise
 */
static int list_contains_ibeacon(struct ibeacon_packet *ibeacon)
{
    struct ibeacon_packet *tmp;

    int ret = k_mutex_lock(&ibeacon_list_mutex, K_MSEC(1));
    if (ret != 0) {
        return ret;
    }

    /* Check if the ibeacon is already in the list */
    SYS_SLIST_FOR_EACH_CONTAINER(&ibeacon_list, tmp, next) {
        
        if (ibeacon->beacon.id == tmp->beacon.id) {

            k_mutex_unlock(&ibeacon_list_mutex);
            return EEXIST;
        }
    }

    k_mutex_unlock(&ibeacon_list_mutex);
    return 0;
}

/**
 * @brief Adds given ibeacon to ibeacon_list
 * 
 * @param ibeacon ibeacon to add to list
 * @return 0 on succes, else negative error code
 */
static int add_ibeacon_to_list(struct ibeacon_packet *ibeacon)
{
    struct ibeacon_packet *tmp;
    int ret;

    ret = k_mutex_lock(&ibeacon_list_mutex, K_MSEC(1));
    if (ret != 0) {
        return ret;
    }

    /* Allocate memory for ibeacon_packet */
    ret = k_mem_slab_alloc(&ibeacon_mem, (void **)&tmp, K_NO_WAIT);
    if (ret != 0) {
        LOG_ERR("add_ibeacon_to_list: Packet couldn't be allocated. (%d)", ret);

        k_mutex_unlock(&ibeacon_list_mutex);
        return ret;
    }

    /* Copy ibeacon_packet to allocated memory */
    memcpy(tmp, ibeacon, sizeof(struct ibeacon_packet));

    /* Add ibeacon to list */
    sys_slist_append(&ibeacon_list, &tmp->next);
    
    k_mutex_unlock(&ibeacon_list_mutex);
    return 0;
}

/**
 * @brief Filter an ibeacon based on if it belongs to a supported network
 */
static void filter_ibeacon(struct k_work *work)
{
    struct ibeacon_packet ibeacon;

    if (k_msgq_get(&ibeacon_msgq, &ibeacon, K_NO_WAIT) != 0) {
        return;
    }

    if (!is_supported_network(&ibeacon.beacon.network.uuid)) {
        return;
    }

    /* ibeacon is part of a supported network, so add to list if not already */
    if (list_contains_ibeacon(&ibeacon) == 0) {
        if (add_ibeacon_to_list(&ibeacon) != 0) {
            return;
        }
        
        LOG_INF("filter_ibeacon: Added beacon: id: %d to list", 
                ibeacon.beacon.id);
    }
}

/**
 * @brief Parses the input data to see if it is an iBeacon packet. If it is,
 *        then fill the user_data struct. If it isn't set user_data to NULL.
 */
static bool parse_ad(struct bt_data *data, void *user_data)
{
    struct ibeacon_packet *ad_data = user_data;
    int ret;
 
    /* 
     * iBeacon has a BT_DATA_FLAGS and BT_DATA_MANUFACTURER section, so allow
     * the parser to check the next section 
     */
    if (data->type == BT_DATA_FLAGS)
        return true;

    /* iBeacons only have manufacturer data with data length 25 */
    if (data->type != BT_DATA_MANUFACTURER_DATA || data->data_len != 25) {
        user_data = NULL;
        return false;
    }

    /* iBeacons packets always start with 0x004c0215 */
    if (sys_cpu_to_be32(((uint32_t *)data->data)[0]) != 0x004c0215) {
        user_data = NULL;
        return false;
    }

    /* Save UUID in correct endianness */
    sys_mem_swap((void *)&data->data[4], 16);
    ret = bt_uuid_create(&ad_data->beacon.network.uuid, &data->data[4], 
            BT_UUID_SIZE_128);
    if (!ret) {
        user_data = NULL;
        return false;
    }

    /* Ensure endianness */
    uint16_t major = sys_be16_to_cpu(*(uint16_t *)&data->data[20]);
    uint16_t minor = sys_be16_to_cpu(*(uint16_t *)&data->data[22]);
    
    ad_data->beacon.id = BEACON_ID_INIT(major, minor);
    ad_data->tx_power = data->data[24];

    LOG_DBG("id: %d, tx: %d", ad_data->beacon.id, ad_data->tx_power);

    return false;
}

/**
 * @brief Callback for when a BLE advertisement is scanned. Checks if the 
 *        packet is an iBeacon packet and sends it to the queue if it is.
 */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, 
        struct net_buf_simple *ad)
{
    struct ibeacon_packet *beacon_data, beacon_data_tmp = {0};
    struct timespec discovered_time;

    beacon_data = &beacon_data_tmp;

    /* Only want non-connectable and non-scannable advertisements (iBeacon) */
    if (adv_type != BT_GAP_ADV_TYPE_ADV_NONCONN_IND || rssi < -70)
        return;

    /* Parse the data to see if it is an iBeacon */
    bt_data_parse(ad, parse_ad, beacon_data);    
    if (beacon_data == NULL)
        return;

    clock_gettime(CLOCK_REALTIME, &discovered_time);

    /* Save RSSI and the time the packet was received */
    beacon_data->rssi = rssi;
    beacon_data->discovered_time = discovered_time.tv_sec;

    /* Add the ibeacon to the queue and submit filter work */
    k_msgq_put(&ibeacon_msgq, beacon_data, K_MSEC(10));
    k_work_submit(&ibeacon_filter_work);
}

/**
 * @brief Thread to perform localisation using iBeacons 
 */
void ble_localisation(void *a, void *b, void *c)
{
    int ret, beacons_found = 0, missed_scans = 0;
    struct ibeacon_packet *beacon;
    sys_snode_t *node;

    sys_slist_init(&ibeacon_list);
    k_mutex_init(&ibeacon_list_mutex);

    k_work_init(&ibeacon_filter_work, filter_ibeacon);

    ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("Bluetooth init failed (err %d)", ret);
        return;
    }

    LOG_DBG("Bluetooth initialised");

    ret = bt_le_scan_start(&scan_params, scan_cb);
    if (ret) {
        LOG_ERR("BLE scanning failed (err %d)", ret);
        return;
    }

    while (1) {
        k_msleep(SCAN_PERIOD);

        LOG_WRN("Scan period elapsed. Missed scans %d", missed_scans);

        /* If enough beacons have been found then run the localisation algo */
        beacons_found = k_mem_slab_num_used_get(&ibeacon_mem);
        if (beacons_found < 3) {
            ++missed_scans;

            /* Missed too many scans. Stop scanning for 10 minutes */
            if (missed_scans >= MAX_MISSED_SCAN_PERIODS) {

                bt_le_scan_stop();

                /* Scanning stopped, so empty the list */
                empty_ibeacon_list();

                LOG_INF("Missed %d scanning periods, stopped iBeacon "
                        "scanning and suspending", missed_scans);

                k_sleep(K_MINUTES(10));

                bt_le_scan_start(&scan_params, scan_cb);

                missed_scans = 0;
            }
        } else {
            missed_scans = 0;

            k_mutex_lock(&ibeacon_list_mutex, K_MSEC(1));
            while ((node = sys_slist_get(&ibeacon_list)) != NULL) {
                beacon = SYS_SLIST_CONTAINER(node, beacon, next);
                
                // DEBUG
                LOG_INF("Beacon ID %u, rssi %d", beacon->beacon.id, 
                        beacon->rssi);
                
                // TODO: Find a position estimate

                k_mem_slab_free(&ibeacon_mem, (void **)&beacon);
            }
            k_mutex_unlock(&ibeacon_list_mutex);

            struct location location = {
                .altitude = 1000,
                .latitude = 2000,
                .longitude = 213,
            };

            /* Send environmental data */
            while (k_msgq_put(&location_msgq, &location, K_NO_WAIT) != 0) {
                /* message queue is full: purge old data & try again */
                k_msgq_purge(&location_msgq);
                LOG_DBG("location_msgq has been purged");
            }

            k_event_post(&data_events, LOCATION_DATA_PENDING);
        }
    }
}

K_THREAD_DEFINE(ble_localisation_id, BLE_LOCALISATION_STACK_SIZE, 
        ble_localisation, NULL, NULL, NULL, BLE_LOCALISATION_PRIORITY, 
        0, BLE_LOCALISATION_STARTUP_DELAY);