#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "file/file_common.h"
#include "file/file_init.h"


LOG_MODULE_REGISTER(controller);


#define CONTROLLER_STACK_SIZE  2048
#define CONTROLLER_PRIORITY    3


static const struct gpio_dt_spec bat_chg_inidicator = GPIO_DT_SPEC_GET(
        DT_NODELABEL(bat_chrg), gpios);

static struct gpio_callback battery_charge_cb_data;


static void charging_inidicator_change(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    static bool is_charging = false;

    is_charging = !is_charging;
    LOG_INF("Battery %s charging", (is_charging) ? "is" : "has stopped");
}

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
    int ret;


    LOG_INF("Controller started");

    #ifdef CONFIG_APP_INIT_BEACON_FILES
    initialise_files();
    #endif /* CONFIG_APP_INIT_BEACON_FILES */

    increment_boot_count();

    /* Configure battery charging indicator */
    if (!device_is_ready(bat_chg_inidicator.port)) {
        LOG_ERR("CHG indicator is not ready.");
        return;
    }

    ret = gpio_pin_configure_dt(&bat_chg_inidicator, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Failed to configure GPIO on %s pin %d. (%d)", 
                bat_chg_inidicator.port->name, bat_chg_inidicator.pin, ret);
        return;
    }

    ret = gpio_pin_interrupt_configure_dt(&bat_chg_inidicator, 
            GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        LOG_ERR("Failed to configure interrupt on %s pin %d. (%d)", 
                bat_chg_inidicator.port->name, bat_chg_inidicator.pin, ret);
        return;
    }

    gpio_init_callback(&battery_charge_cb_data, charging_inidicator_change, 
            BIT(bat_chg_inidicator.pin));
    gpio_add_callback(bat_chg_inidicator.port, &battery_charge_cb_data);


	while (1) {
		k_msleep(100);
	}
}

K_THREAD_DEFINE(controller_id, CONTROLLER_STACK_SIZE, controller_thread, NULL,
		NULL, NULL, CONTROLLER_PRIORITY, 0, 0);