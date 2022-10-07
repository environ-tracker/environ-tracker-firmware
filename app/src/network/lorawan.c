#include <zephyr/device.h>
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "accumulator.h"
#include "ble_localisation.h"
#include "ble/ble_network.h"
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


static struct k_poll_event events[] = {
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
            K_POLL_MODE_NOTIFY_ONLY, &lorawan_msgq, 0),
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE, 
            K_POLL_MODE_NOTIFY_ONLY, &ibeacon_request_msgq, 0),
};


static void dl_callback(uint8_t port, bool data_pending,
        int16_t rssi, int8_t snr, uint8_t len, const uint8_t *data)
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
    struct ibeacon ibeacon = {0};

    
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

    uint8_t attempts = 0;
    do {
        /* Join LoRaWAN network */
        LOG_INF("Joining network over OTAA");
        ret = lorawan_join(&join_cfg);
        if (ret < 0) {
            LOG_WRN("lorawan_join failed: %d", ret);
        
            if (++attempts > 5) {
                LOG_ERR("Failed to join LoRaWAN network after %d attempts. "
                        "Exiting.", attempts);
                return;
            }
            k_sleep(K_SECONDS(30));
        }
    } while (ret != 0);


    while (1) {
        ret = k_poll(events, 2, K_FOREVER);
        if (ret != 0) {
            continue;
        }

        if (events[0].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
            events[0].state = K_POLL_STATE_NOT_READY;

            /* Receive and encode system data */
            ret = k_msgq_get(&lorawan_msgq, &sys_data, K_MSEC(10));
            if (ret) {
                continue;
            }

            if (!encode_sys_data_message(tx_buffer, sizeof(tx_buffer), 
                    &data_len, &sys_data)) {
                continue;
            }
        } else if (events[1].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
            events[1].state = K_POLL_STATE_NOT_READY;

            /* Request ibeacon location data */
            ret = k_msgq_get(&ibeacon_request_msgq, &ibeacon, K_MSEC(10));
            if (ret) {
                continue;
            }

            if (!encode_ibeacon_request_message(tx_buffer, sizeof(tx_buffer), 
                    &data_len, &ibeacon)) {
                continue;
            }
        } else {
            continue;
        }

        LOG_INF("data encoded successfully, using %d bytes, sending...", 
                data_len);
        ret = lorawan_send(2, tx_buffer, data_len, LORAWAN_MSG_CONFIRMED);

        if (ret < 0) {
            LOG_ERR("lorawan_send failed: %d", ret);
            k_sleep(K_SECONDS(30));
            continue;
        }

        LOG_INF("Data sent!");
        
        k_sleep(K_MINUTES(5));
    }
}

K_THREAD_DEFINE(lorawan_backend_id, LORAWAN_BACKEND_STACK_SIZE, 
        lorawan_backend, NULL, NULL, NULL, LORAWAN_BACKEND_PRIORITY, 0, 0);