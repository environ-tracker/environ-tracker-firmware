#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(ble_localisation);


static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, 
        struct net_buf_simple *ad)
{
    // TODO do tings
    LOG_INF("rssi %d, adv_type %d", rssi, adv_type);
}

void ble_localisation(void *a, void *b, void *c)
{
    int ret;
    struct bt_le_scan_param scan_params = {
        .type = BT_HCI_LE_SCAN_PASSIVE,
        .options = BT_LE_SCAN_OPT_NONE,
        .interval = 0x0040,
        .window = 0x0040
    };

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
        k_msleep(K_MSEC(1000));
        // TODO: Process all found
    }

}