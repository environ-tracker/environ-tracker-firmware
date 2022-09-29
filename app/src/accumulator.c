#include <zephyr.h>
#include <logging/log.h>

#include "accumulator.h"
#include "environ.h"


LOG_MODULE_REGISTER(accumulator);


#define ACCUMULATOR_STACK_SIZE  2048
#define ACCUMULATOR_PRIORITY    6


// Create the message queue for the LoRaWAN backend
K_MSGQ_DEFINE(lorawan_msgq, sizeof(struct system_data), 5, 4);


/**
 * @brief Thread to accumulate and aggregate all system data to then forward to
 *        LoRaWAN send and OLED display threads as required.
 */
void accumulator_thread(void *a, void *b, void *c)
{
    struct environ_data data = {0};    
    struct system_data sys_data = {0};
    int err;
    bool lorawan = true;

    LOG_INF("Accumulator started");


    while (1) {

        err = k_msgq_get(&environ_data_msgq, &data, K_FOREVER);
        if (err == 0) {
            LOG_DBG("environ data received:\r\n\t"
                    "T: %d.%06d; P: %d.%06d; H: %d.%06d; G: %d.%06d; "
                    "L: %d; IR: %d; UV: %d.%06d", data.temp.val1, 
                    data.temp.val2, data.press.val1, data.press.val2, 
                    data.humidity.val1, data.humidity.val2, data.gas_res.val1, 
                    data.gas_res.val2, data.vis_light.val1, data.ir_light.val1,
                    data.uv_index.val1, data.uv_index.val2);
        } else {
            LOG_ERR("environ data receive error: %d", err);
        }


        if (lorawan) {
            sys_data.placeholder = 10;

            // Send to lorawan_thread
            while (k_msgq_put(&lorawan_msgq, &sys_data, K_MSEC(2)) != 0) {
                k_msgq_purge(&lorawan_msgq);
                LOG_DBG("lorawan_msgq has been purged");
            }
        }

    }
}

K_THREAD_DEFINE(accumulator_id, ACCUMULATOR_STACK_SIZE, accumulator_thread, NULL, NULL, NULL, ACCUMULATOR_PRIORITY, 0, 0);