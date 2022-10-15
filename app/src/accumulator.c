#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "accumulator.h"
#include "environ.h"
#include "imu.h"
#include "location.h"

LOG_MODULE_REGISTER(accumulator);


#define ACCUMULATOR_STACK_SIZE  2048
#define ACCUMULATOR_PRIORITY    6

/* Only send environ tracker data packets every 30s */
#define LORAWAN_SEND_PERIOD     30 * 1000

/* Create the message queue for the GUI backend */
K_MSGQ_DEFINE(gui_msgq, sizeof(struct system_data), 5, 4);

/* Create the message queue for the LoRaWAN backend */
K_MSGQ_DEFINE(lorawan_msgq, sizeof(struct system_data), 5, 4);

/* Create message queue to receive location data from */
K_MSGQ_DEFINE(location_msgq, sizeof(struct location_wrapper), 5, 4);

/* Input data pending event group */
K_EVENT_DEFINE(data_events);


/**
 * @brief Thread to accumulate and aggregate all system data to then forward to
 *        LoRaWAN send and OLED display threads as required.
 */
void accumulator_thread(void *a, void *b, void *c)
{
    struct system_data sys_data = {0};
    int64_t last_lorawan_send = 0;
    uint32_t events;
    int err;
    

    LOG_INF("Accumulator started");


    while (1) {
        /* Wait for all data to be ready */
        events = k_event_wait(&data_events, ENVIRON_DATA_PENDING |
                LOCATION_DATA_PENDING, false, K_SECONDS(10));

        if (events & ENVIRON_DATA_PENDING) {
            k_event_set_masked(&data_events, 0, ENVIRON_DATA_PENDING);

            err = k_msgq_get(&environ_data_msgq, &sys_data.environ, K_NO_WAIT);
            if (err != 0) {
                LOG_ERR("environ data receive error: %d", err);
                continue;
            }
        } else if (events & LOCATION_DATA_PENDING) {
            k_event_set_masked(&data_events, 0, LOCATION_DATA_PENDING);

            err = k_msgq_get(&location_msgq, &sys_data.location, K_NO_WAIT);
            if (err != 0) {
                LOG_ERR("location receive error: %d", err);
                continue;
            }
        }

        LOG_DBG("Sending sys_data");

        /* Some data has changed, so send it onwards */
        if (events) {
            /* Send to lorawan_thread */
            if ((k_uptime_get() - last_lorawan_send) > LORAWAN_SEND_PERIOD) {
                last_lorawan_send = k_uptime_get();
                
                while (k_msgq_put(&lorawan_msgq, &sys_data, K_MSEC(2)) != 0) {
                    k_msgq_purge(&lorawan_msgq);
                    LOG_DBG("lorawan_msgq has been purged");
                }
            }

            /* Send to GUI thread */
            while (k_msgq_put(&gui_msgq, &sys_data, K_MSEC(2)) != 0) {
                k_msgq_purge(&gui_msgq);
                LOG_DBG("gui_msgq has been purged");
            }
        }
    }
}

K_THREAD_DEFINE(accumulator_id, ACCUMULATOR_STACK_SIZE, accumulator_thread, NULL, NULL, NULL, ACCUMULATOR_PRIORITY, 0, 0);