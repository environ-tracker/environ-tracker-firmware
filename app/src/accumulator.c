#include <zephyr.h>
#include <logging/log.h>

#include "environ.h"

LOG_MODULE_REGISTER(accumulator);

#define ACCUMULATOR_STACK_SIZE  500
#define ACCUMULATOR_PRIORITY    5

void accumulator_thread(void *a, void *b, void *c)
{
    struct environ_data data = {0};
    int err;

    LOG_INF("Accumulator started");

    while (1) {

        err = k_msgq_get(&environ_data_msgq, &data, K_FOREVER);
        if (err == 0) {
            LOG_INF("environ data received:\r\n\t"
                    "T: %d.%06d; P: %d.%06d; H: %d.%06d; G: %d.%06d; "
                    "L: %d; IR: %d; UV: %d.%06d", data.temp.val1, 
                    data.temp.val2, data.press.val1, data.press.val2, 
                    data.humidity.val1, data.humidity.val2, data.gas_res.val1, 
                    data.gas_res.val2, data.vis_light.val1, data.ir_light.val1,
                    data.uv_index.val1, data.uv_index.val2);
        } else {
            LOG_ERR("envorn data receive error: %d", err);
        }
    }
}

K_THREAD_DEFINE(accumulator_id, ACCUMULATOR_STACK_SIZE, accumulator_thread, NULL, NULL, NULL, ACCUMULATOR_PRIORITY, 0, 0);