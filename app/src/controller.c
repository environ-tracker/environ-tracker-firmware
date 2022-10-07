#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "file/file_common.h"
#include "file/file_init.h"


LOG_MODULE_REGISTER(controller);


#define CONTROLLER_STACK_SIZE  2048
#define CONTROLLER_PRIORITY    3


/**
 * @brief Increment the systems boot counter
 * 
 * @return 0 on success, else negative error code from FS API
 */
int increment_boot_count(void)
{
    uint32_t boot_count = 0;
    int err;

    err = read_file("boot_count", (uint8_t *)&boot_count, sizeof(boot_count));
	if (err == -EEXIST) {
		boot_count = 0;
	} else if (err != 0) {
        return err;
    } else {
		LOG_INF("boot_count is: %d", boot_count);
		++boot_count;
	}

	err = write_file("boot_count", (uint8_t *)&boot_count, sizeof(boot_count));
	
    return err;
}


void controller_thread(void *a, void *b, void *c)
{
    LOG_INF("Controller started");

    #ifdef CONFIG_APP_INIT_BEACON_FILES
    initialise_files();
    #endif /* CONFIG_APP_INIT_BEACON_FILES */

    increment_boot_count();

	while (1) {
		k_msleep(100);
	}
}

K_THREAD_DEFINE(controller_id, CONTROLLER_STACK_SIZE, controller_thread, NULL,
		NULL, NULL, CONTROLLER_PRIORITY, 0, 0);