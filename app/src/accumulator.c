#include <zephyr.h>
#include <logging/log.h>

#include "environ.h"
#include "file/file_common.h"
#include "ble/ble_network.h"
#include "ble/ble_helpers.h"

LOG_MODULE_REGISTER(accumulator);

#define ACCUMULATOR_STACK_SIZE  2048
#define ACCUMULATOR_PRIORITY    6


void accumulator_thread(void *a, void *b, void *c)
{
    struct environ_data data = {0};
    uint32_t boot_count = 0;
    
    int err;

    LOG_INF("Accumulator started");

	// absolute_file_name(fname, "boot_count");

	// TODO: move boot_count incrementing to a different place
	err = read_file("boot_count", (uint8_t *)&boot_count, sizeof(boot_count));
	if (err != 0) {
		boot_count = 0;
	} else {
		LOG_INF("boot_count is: %d", boot_count);
		++boot_count;
	}

	err = write_file("boot_count", (uint8_t *)&boot_count, sizeof(boot_count));
	if (err != 0) {
		LOG_ERR("something");
	}
	// TODO: 


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
    }
}

K_THREAD_DEFINE(accumulator_id, ACCUMULATOR_STACK_SIZE, accumulator_thread, NULL, NULL, NULL, ACCUMULATOR_PRIORITY, 0, 0);