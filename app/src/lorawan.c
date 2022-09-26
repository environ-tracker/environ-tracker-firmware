#include <device.h>
#include <lorawan/lorawan.h>
#include <zephyr.h>
#include <logging/log.h>


LOG_MODULE_REGISTER(lorawan_backend, LOG_LEVEL_INF);

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
        "No default LoRa radio specified in DT");
#define DEFAULT_RADIO DT_LABEL(DEFAULT_RADIO_NODE)

#define LORAWAN_DEV_EUI     {0x71, 0xb3, 0xd5, 0x7e, 0xd0, 0x05, 0x07, 0x43}

#define LORAWAN_JOIN_EUI    {0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

#define LORAWAN_APP_KEY     {0xad, 0x16, 0x8e, 0xb9, 0x0f, 0xc2, 0x4b, 0x7e,\
							0x97, 0x7e, 0x88, 0xd8, 0x7e, 0x04, 0xb8, 0x74}

#define LORAWAN_BACKEND_STACK_SIZE  2048
#define LORAWAN_BACKEND_PRIORITY    7


char data[] = {'h', 'e', 'l', 'l', 'o', 'w', 'o', 'r', 'l', 'd'};

static void dl_callback(uint8_t port, bool data_pending,
            int16_t rssi, int8_t snr,
            uint8_t len, const uint8_t *data)
{
    LOG_INF("Port %d, Pending %d, RSSI %ddB, SNR %ddBm", port, data_pending, rssi, snr);
    if (data) {
        LOG_HEXDUMP_INF(data, len, "Payload: ");
    }
}

/**
 * @brief Fetches all required data and then sends it to the network
 * 
 * NOTE: Currently this just sends "helloworld" to the network every 10s
 */
void lorawan_backend(void *a, void *b, void *c)
{
    const struct device *lora_dev;
    struct lorawan_join_config join_cfg;
    uint8_t dev_eui[] = LORAWAN_DEV_EUI;
    uint8_t join_eui[] = LORAWAN_JOIN_EUI;
    uint8_t app_key[] = LORAWAN_APP_KEY;
    int ret;

    struct lorawan_downlink_cb downlink_cb = {
        .port = LW_RECV_PORT_ANY,
        .cb = dl_callback
    };

    lora_dev = device_get_binding(DEFAULT_RADIO);
    if (!lora_dev) {
        LOG_ERR("%s Device not found", DEFAULT_RADIO);
        return;
    }

    ret = lorawan_start();
    if (ret < 0) {
        LOG_ERR("lorawan_start failed: %d", ret);
        return;
    }

    lorawan_register_downlink_callback(&downlink_cb);

    join_cfg.mode = LORAWAN_ACT_OTAA;
    join_cfg.dev_eui = dev_eui;
    join_cfg.otaa.join_eui = join_eui;
    join_cfg.otaa.app_key = app_key;
    join_cfg.otaa.nwk_key = app_key;

    LOG_INF("Joining network over OTAA");
    ret = lorawan_join(&join_cfg);
    if (ret < 0) {
        LOG_ERR("lorawan_join_network failed: %d", ret);
        return;
    }

    LOG_INF("Sending data...");
    while (1) {
        ret = lorawan_send(2, data, sizeof(data), LORAWAN_MSG_CONFIRMED);

        /*
         * Note: The stack may return -EAGAIN if the provided data
         * length exceeds the maximum possible one for the region and
         * datarate. But since we are just sending the same data here,
         * we'll just continue.
         */
        if (ret == -EAGAIN) {
            LOG_ERR("lorawan_send failed: %d. Continuing...", ret);
            k_sleep(K_MSEC(10000));
            continue;
        }

        if (ret < 0) {
            LOG_ERR("lorawan_send failed: %d", ret);
            return;
        }

        LOG_INF("Data sent!");
        k_sleep(K_MSEC(10000));
    }
}

K_THREAD_DEFINE(lorawan_backend_id, LORAWAN_BACKEND_STACK_SIZE, 
        lorawan_backend, NULL, NULL, NULL, LORAWAN_BACKEND_PRIORITY, 0, 0);