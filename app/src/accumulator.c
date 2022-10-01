#include <zephyr.h>
#include <posix/time.h>
#include <logging/log.h>

#include "accumulator.h"
#include "environ.h"


LOG_MODULE_REGISTER(accumulator);


#define ACCUMULATOR_STACK_SIZE  2048
#define ACCUMULATOR_PRIORITY    6


// Create the message queue for the LoRaWAN backend
K_MSGQ_DEFINE(lorawan_msgq, sizeof(struct system_data), 5, 4);

/* Input data pending event group */
K_EVENT_DEFINE(data_events);


/**
 * @brief Thread to accumulate and aggregate all system data to then forward to
 *        LoRaWAN send and OLED display threads as required.
 */
void accumulator_thread(void *a, void *b, void *c)
{
    struct environ_data data = {0};    
    struct system_data sys_data = {0};
    struct timespec time = {0};
    int err;
    bool lorawan = true;

    LOG_INF("Accumulator started");


    while (1) {
        /* Wait for all data to be ready */
        // k_event_wait_all(&data_events, ENVIRON_DATA_PENDING | 
        //         ACTIVITY_DATA_PENDING | LOCATION_DATA_PENDING, true, 
        //         K_FOREVER);

        k_event_wait(&data_events, ENVIRON_DATA_PENDING | 
                ACTIVITY_DATA_PENDING | LOCATION_DATA_PENDING, true, 
                K_FOREVER);

        err = k_msgq_get(&environ_data_msgq, &data, K_NO_WAIT);
        if (err != 0) {
            LOG_ERR("environ data receive error: %d", err);
            continue;
        }


        clock_gettime(CLOCK_REALTIME, &time);

        if (lorawan) {
            sys_data.timestamp = time.tv_sec;
            sys_data.activity = ACTIVITY_UNDEFINED;
            sys_data.environ = data;
            sys_data.location.longitude = 102031;
            sys_data.location.latitude = 102031;
            sys_data.location.altitude = 102031;

            // Send to lorawan_thread
            while (k_msgq_put(&lorawan_msgq, &sys_data, K_MSEC(2)) != 0) {
                k_msgq_purge(&lorawan_msgq);
                LOG_DBG("lorawan_msgq has been purged");
            }
        }

    }
}

K_THREAD_DEFINE(accumulator_id, ACCUMULATOR_STACK_SIZE, accumulator_thread, NULL, NULL, NULL, ACCUMULATOR_PRIORITY, 0, 0);