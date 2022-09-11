#define DT_DRV_COMPAT microcrystal_rv3028

#include <zephyr/device.h>
#include <zephyr/drivers/counter>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/rtc/rtc_rv3028.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rv3028, CONFIG_COUNTER_LOG_LEVEL);

