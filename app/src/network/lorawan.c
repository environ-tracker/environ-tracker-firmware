#include <device.h>
#include <lorawan/lorawan.h>
#include <zephyr.h>
#include <logging/log.h>

#include "accumulator.h"
#include "encode_pb.h"
#include "src/network/proto/upload_data.pb.h"


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
 */
void lorawan_backend(void *a, void *b, void *c)
{
    /* Buffers for LoRaWAN keys */
    uint8_t dev_eui[] = LORAWAN_DEV_EUI;
    uint8_t join_eui[] = LORAWAN_JOIN_EUI;
    uint8_t app_key[] = LORAWAN_APP_KEY;


    uint8_t tx_buffer[EnvironTrackerUpload_size];
    int ret;
    size_t data_len;

    struct system_data sys_data = {0};

    
    /* Get device pointer for LoRa transceiver */
    const struct device *lora_dev = device_get_binding(DEFAULT_RADIO);
    if (!lora_dev) {
        LOG_ERR("%s Device not found", DEFAULT_RADIO);
        return;
    }

    /* Start LoRaWAN stack */
    ret = lorawan_start();
    if (ret < 0) {
        LOG_ERR("lorawan_start failed: %d", ret);
        return;
    }

    /* Set LoRaWAN to DR3, allowing ~53 payload bytes */
    ret = lorawan_set_datarate(LORAWAN_DR_3);
    if (ret < 0) {
        LOG_ERR("lorawan_set_datarate failed: %d", ret);
        return;
    }

    /* Setup and register callback for downlink messages */
    struct lorawan_downlink_cb downlink_cb = {
        .port = LW_RECV_PORT_ANY,
        .cb = dl_callback
    };

    lorawan_register_downlink_callback(&downlink_cb);

    /* Setup LoRaWAN join config */
    struct lorawan_join_config join_cfg = {
        .mode = LORAWAN_ACT_OTAA,
        .dev_eui = dev_eui,
        .otaa = {
            .join_eui = join_eui,
            .app_key = app_key,
            .nwk_key = app_key,
        },
    };

    /* Join LoRaWAN network */
    LOG_INF("Joining network over OTAA");
    ret = lorawan_join(&join_cfg);
    if (ret < 0) {
        LOG_ERR("lorawan_join failed: %d", ret);
        return;
    }


    while (1) {
        ret = k_msgq_get(&lorawan_msgq, &sys_data, K_FOREVER);
        if (ret == 0) {
            LOG_INF("sys_data received, timestamp: %d", sys_data.timestamp);

            if (!encode_message(tx_buffer, sizeof(tx_buffer), &data_len, 
                    &sys_data)) {
                k_msleep(5000); 
                continue;    
            } 

            LOG_INF("data encoded successfully, using %d bytes, sending...", 
                    data_len);
            ret = lorawan_send(2, tx_buffer, data_len, 
                    LORAWAN_MSG_CONFIRMED);

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
            } else if (ret < 0) {
                LOG_ERR("lorawan_send failed: %d", ret);
                k_msleep(5000);
                continue;
            }

            LOG_INF("Data sent!");
        } else {
            LOG_ERR("message queue receive error: %d", ret);
        }
    
        k_sleep(K_MSEC(10000));
    }
}

K_THREAD_DEFINE(lorawan_backend_id, LORAWAN_BACKEND_STACK_SIZE, 
        lorawan_backend, NULL, NULL, NULL, LORAWAN_BACKEND_PRIORITY, 0, 0);