#include "ble_localisation.h"

#include <zephyr.h>
#include <kernel.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(ble_localisation, LOG_LEVEL_INF);


#define MAX_LOCALISATION_BEACONS    16

#define SCAN_PERIOD     5000
#define MAX_MISSED_SCAN_PERIODS 5

#define BLE_LOCALISATION_STACK_SIZE 1024
#define BLE_LOCALISATION_PRIORITY   5

/* This delay is so that BLE logs can be seen from the shell */
#define BLE_LOCALISATION_STARTUP_DELAY 500


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
    struct sys_snode_t *node;

    k_mutex_lock(&ibeacon_list_mutex, K_MSEC(1));
    while ((node = sys_slist_get(&ibeacon_list)) != NULL) {
        beacon = SYS_SLIST_CONTAINER(node, beacon, next);

        k_mem_slab_free(&ibeacon_mem, &beacon);
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
    int index = 0;

    /* iBeacons only have manufacturer data with data length 25 */
    if ((data->type != BT_DATA_MANUFACTURER_DATA) && data->data_len != 25) {
        user_data = NULL;
        return false;
    }

    /* iBeacon packets have Company ID = 0x004c and Beacon Type = 0x0215 */
    if (((uint32_t *)(data->data))[index] != 0x004c0215) {
        user_data = NULL;
        return false;
    }

    index += 4;

    if (!bt_uuid_create((struct bt_uuid *)&ad_data->uuid, &data->data[index], 16))
        LOG_ERR("Problem occured while saving UUID from AD data");

    index += 16;

    ad_data->major = *(uint16_t *)&data->data[index];

    index += 2;

    ad_data->minor = *(uint16_t *)&data->data[index];

    index += 2;

    ad_data->tx_power = (int8_t)data->data[index];

    return false;
}

/**
 * @brief Callback for when a BLE advertisement is scanned. Checks if the 
 *        packet is an iBeacon packet and sends it to the queue if it is.
 */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, 
        struct net_buf_simple *ad)
{
    struct ibeacon_data *beacon_data, *tmp;
    int ret;

    /* Only want non-connectable and non-scannable advertisements (iBeacon) */
    if (adv_type != BT_GAP_ADV_TYPE_ADV_NONCONN_IND || rssi < -70)
        return;

    ret = k_mem_slab_alloc(&ibeacon_mem, &beacon_data, K_NO_WAIT);
    if (ret != 0) {
        LOG_ERR("Beacon packet couldn't be allocated (ret %d)", ret);
        empty_ibeacon_list();
        return;
    }

    LOG_INF("rssi %d, adv_type %d", rssi, adv_type);

    bt_data_parse(ad, parse_ad, beacon_data);
    if (beacon_data == NULL) {
        LOG_INF("AD wasn't an iBeacon packet");
        return;
    }

    /* Save RSSI and the time the packet was received */
    beacon_data->rssi = rssi;
    beacon_data->discovered_time = k_uptime_get();

    /* Check if the ibeacon is already in the list */
    k_mutex_lock(&ibeacon_list_mutex, K_NO_WAIT);
    SYS_SLIST_FOR_EACH_CONTAINER(&ibeacon_list, tmp, next) {
        
        if (beacon_data->major == tmp->major && beacon_data->minor == tmp->minor) {
            LOG_INF("Found matching major and minor");

            k_mem_slab_free(&ibeacon_mem, &beacon_data);
            k_mutex_unlock(&ibeacon_list_mutex);
            return;
        }
    }

    /* ibeacon wasn't in the list, so append it */
    sys_slist_append(&ibeacon_list, &beacon_data->next);
    k_mutex_unlock(&ibeacon_list_mutex);
}

/**
 * @brief Restarts BLE advertisement scanning
 */
void scan_restart_work_handler(struct k_work *work)
{
    LOG_INF("Scan restart timer elapsed, restarting iBeacon scanning");

    bt_le_scan_start(&scan_params, scan_cb);
    k_thread_resume(ble_localisation_id);
}

/* Defines the k_work to restart scanning */
K_WORK_DEFINE(scan_restart, scan_restart_work_handler);

/**
 * @brief Timer callback to restart BLE scanning
 */
static void scan_restart_handler(struct k_timer *timer)
{
    k_work_submit(&scan_restart);
}

/* Timer to restart scanning */
K_TIMER_DEFINE(scan_restart_timer, scan_restart_handler, NULL);

/**
 * @brief Thread to perform localisation using iBeacons 
 */
void ble_localisation(void *a, void *b, void *c)
{
    int ret, beacons_found=0, missed_scans = 0;
    struct ibeacon_data *beacon;
    struct sys_snode_t *node;

    sys_slist_init(&ibeacon_list);

    ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("Bluetooth init failed (err %d)", ret);
        return;
    }

    LOG_INF("Bluetooth initialised");

    ret = bt_le_scan_start(&scan_params, scan_cb);
    if (ret) {
        LOG_ERR("BLE scanning failed (err %d)", ret);
        return;
    }

    while (1) {
        k_msleep(SCAN_PERIOD);

        LOG_WRN("Scan period elapsed. Missed scans %d", missed_scans);

        beacons_found = k_mem_slab_num_used_get(&ibeacon_mem);
        if (beacons_found < 3) {
            ++missed_scans;

            /* Missed too many scans. Stop scanning for 10 minutes */
            if (missed_scans >= MAX_MISSED_SCAN_PERIODS) {

                bt_le_scan_stop();

                /* Scanning stopped, so empty the list */
                empty_ibeacon_list();

                k_timer_start(&scan_restart_timer, K_MINUTES(10), K_FOREVER);

                LOG_INF("Missed %d scanning periods, stopped iBeacon "
                        "scanning and suspending", missed_scans);

                k_thread_suspend(ble_localisation_id);
            }
        } else {
            missed_scans = 0;

            k_mutex_lock(&ibeacon_list_mutex, K_MSEC(1));
            while ((node = sys_slist_get(&ibeacon_list)) != NULL) {
                beacon = SYS_SLIST_CONTAINER(node, beacon, next);
                
                LOG_INF("Beacon major: %u, minor %u, rssi %d", beacon->major, 
                        beacon->minor, beacon->rssi);
                
                k_mem_slab_free(&ibeacon_mem, &beacon);
            }
            k_mutex_unlock(&ibeacon_list_mutex);

            // TODO: Find a position estimate

        }
    }
}

K_THREAD_DEFINE(ble_localisation_id, BLE_LOCALISATION_STACK_SIZE, 
        ble_localisation, NULL, NULL, NULL, BLE_LOCALISATION_PRIORITY, 
        0, BLE_LOCALISATION_STARTUP_DELAY);