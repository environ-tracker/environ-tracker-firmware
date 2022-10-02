#include "ble_localisation.h"

#include <zephyr.h>
#include <kernel.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>

#include <sys/byteorder.h>

#include <logging/log.h>

#include "accumulator.h"
#include "location.h"

LOG_MODULE_REGISTER(ble_localisation, LOG_LEVEL_INF);


#define MAX_LOCALISATION_BEACONS    16

#define SCAN_PERIOD     5000
#define MAX_MISSED_SCAN_PERIODS 5

#define BLE_LOCALISATION_STACK_SIZE 1024
#define BLE_LOCALISATION_PRIORITY   5

/* This delay is so that BLE logs can be seen from the shell */
#define BLE_LOCALISATION_STARTUP_DELAY 500

/* NOTE: This is currently the only supported iBeacon network UUID */
static const struct bt_uuid_128 supported_beacon_network = BT_UUID_INIT_128(
        BT_UUID_128_ENCODE(0x7aaf1b67, 0xf3c0, 0x4a54, 0xb314, 0x58fff1960a40));

/* Stores all required ibeacon data */
struct ibeacon_data {
    int64_t discovered_time;
    struct bt_uuid_128 uuid; 
    uint16_t major;
    uint16_t minor;
    int8_t tx_power;
    int8_t rssi;
    sys_snode_t next;
};

/* Scan parameters for BLE advertisement scanning */
static const struct bt_le_scan_param scan_params = {
        .type = BT_HCI_LE_SCAN_PASSIVE,
        .options = BT_LE_SCAN_OPT_NONE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window = BT_GAP_SCAN_FAST_WINDOW
};

/* List to append ibeacon data to */
static sys_slist_t ibeacon_list;

/* Mutex to protect access to ibeacon_list */
K_MUTEX_DEFINE(ibeacon_list_mutex);

/* Memory slab to allocate ibeacon data from */
K_MEM_SLAB_DEFINE_STATIC(ibeacon_mem, sizeof(struct ibeacon_data), 
        MAX_LOCALISATION_BEACONS, 4);


static void empty_ibeacon_list(void)
{
    struct ibeacon_data *beacon;
    sys_snode_t *node;

    k_mutex_lock(&ibeacon_list_mutex, K_MSEC(1));
    while ((node = sys_slist_get(&ibeacon_list)) != NULL) {
        beacon = SYS_SLIST_CONTAINER(node, beacon, next);

        k_mem_slab_free(&ibeacon_mem, (void **)&beacon);
    }
    k_mutex_unlock(&ibeacon_list_mutex);
}

/**
 * @brief Parses the input data to see if it is an iBeacon packet. If it is,
 *        then fill the user_data struct. If it isn't set user_data to NULL.
 */
static bool parse_ad(struct bt_data *data, void *user_data)
{
    struct ibeacon_data *ad_data = user_data;
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
    ret = bt_uuid_create(&ad_data->uuid.uuid, &data->data[4], 16);
    if (!ret) {
        user_data = NULL;
        return false;
    }

    /* Ensure endianness */
    ad_data->major = sys_cpu_to_be16(*(uint16_t *)&data->data[20]);
    ad_data->minor = sys_cpu_to_be16(*(uint16_t *)&data->data[22]);
    ad_data->tx_power = data->data[24];

    LOG_DBG("maj: %d, min: %d, tx: %d", ad_data->major, ad_data->minor, ad_data->tx_power);

    return false;
}

/**
 * @brief Callback for when a BLE advertisement is scanned. Checks if the 
 *        packet is an iBeacon packet and sends it to the queue if it is.
 */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, 
        struct net_buf_simple *ad)
{
    struct ibeacon_data *beacon_data, *tmp, beacon_data_tmp;
    int ret;

    beacon_data = &beacon_data_tmp;

    /* Only want non-connectable and non-scannable advertisements (iBeacon) */
    if (adv_type != BT_GAP_ADV_TYPE_ADV_NONCONN_IND || rssi < -70)
        return;

    /* Parse the data to see if it is an iBeacon */
    bt_data_parse(ad, parse_ad, beacon_data);    
    if (beacon_data == NULL)
        return;

    /* Save RSSI and the time the packet was received */
    beacon_data->rssi = rssi;
    beacon_data->discovered_time = k_uptime_get();

    /* 
     * See if the network is supported (by checking the UUID), if it is then 
     * the beacon should be in the database 
    */
    if (bt_uuid_cmp(&supported_beacon_network.uuid, &beacon_data->uuid.uuid) != 0)
        return;

    /* Check if the ibeacon is already in the list */
    k_mutex_lock(&ibeacon_list_mutex, K_NO_WAIT);
    SYS_SLIST_FOR_EACH_CONTAINER(&ibeacon_list, tmp, next) {
        
        if (beacon_data->major == tmp->major && 
                beacon_data->minor == tmp->minor) {

            k_mutex_unlock(&ibeacon_list_mutex);
            return;
        }
    }

    // DEBUG
    LOG_INF("New iBeacon");

    // TODO: Maybe handle a non-alloc better??
    ret = k_mem_slab_alloc(&ibeacon_mem, (void **)&beacon_data, K_NO_WAIT);
    if (ret != 0) {
        LOG_ERR("Beacon packet couldn't be allocated (ret %d)", ret);

        k_mutex_unlock(&ibeacon_list_mutex);
        return;
    }

    memcpy(beacon_data, &beacon_data_tmp, sizeof(struct ibeacon_data));

    /* ibeacon wasn't in the list, so append it */
    sys_slist_append(&ibeacon_list, &beacon_data->next);
    k_mutex_unlock(&ibeacon_list_mutex);
}

/**
 * @brief Thread to perform localisation using iBeacons 
 */
void ble_localisation(void *a, void *b, void *c)
{
    int ret, beacons_found = 0, missed_scans = 0;
    struct ibeacon_data *beacon;
    sys_snode_t *node;

    sys_slist_init(&ibeacon_list);

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
                LOG_INF("Beacon major: %u, minor %u, rssi %d", beacon->major, 
                        beacon->minor, beacon->rssi);
                
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