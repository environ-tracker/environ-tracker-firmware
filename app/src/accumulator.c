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
    char fname[FILE_NAME_LEN];
    int err;

    LOG_INF("Accumulator started");

	absolute_file_name(fname, "boot_count");

	// TODO: move boot_count incrementing to a different place
	err = read_file(fname, (uint8_t *)&boot_count, sizeof(boot_count));
	if (err != 0) {
		boot_count = 0;
	} else {
		LOG_INF("boot_count is: %d", boot_count);
		++boot_count;
	}

	err = write_file(fname, (uint8_t *)&boot_count, sizeof(boot_count));
	if (err != 0) {
		LOG_ERR("something");
	}
	// TODO: 


	struct bt_uuid_128 test_uuid;
	char *test = "7aaf1b67-f3c0-4a54-b314-58fff1960a40";
	err = bt_uuid_from_str(&test_uuid.uuid, test);


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

		err = is_supported_network(&test_uuid.uuid);
		LOG_INF("is_supported_network: %d", err);
    }
}

K_THREAD_DEFINE(accumulator_id, ACCUMULATOR_STACK_SIZE, accumulator_thread, NULL, NULL, NULL, ACCUMULATOR_PRIORITY, 0, 0);