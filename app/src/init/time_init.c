#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/posix/time.h>
#include <zephyr/drivers/counter.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(init);

#if !DT_HAS_COMPAT_STATUS_OKAY(microcrystal_rv3028)
#error "Unsupported board: microcrystal,rv3028 not enabled on devicetree"
#endif

#define RTC_NODE DT_LABEL(DT_COMPAT_GET_ANY_STATUS_OKAY(microcrystal_rv3028))

#define TIME_INIT_INIT_PRIORITY 95
#if TIME_INIT_INIT_PRIORITY < CONFIG_SENSOR_INIT_PRIORITY
#error "TIME_INIT_INIT_PRIORITY must be larger than SENSOR_INIT_PRIORITY"
#endif

/**
 * @brief Initialise the POSIX realtime clock by setting it to the external 
 *        RTC's time.
 * 
 * @param dev UNUSED
 * @return 0 on success, else negative error number
 */
int time_init(const struct device *dev)
{
    const struct device *rtc_dev = device_get_binding(RTC_NODE);
    struct timespec time = {0};
    time_t unix_time = 0;
    int rc;

    if (rtc_dev == NULL) {
        LOG_ERR("time_init: No RV3028 RTC device found.");
        return -ENODEV;
    }

    rc = counter_get_value(rtc_dev, (uint32_t *)&unix_time);
    if (rc < 0) {
        LOG_ERR("time_init: Error while getting unix time from RTC. (%d)", rc);
        return rc;
    }

    time.tv_sec = unix_time;

    rc = clock_settime(CLOCK_REALTIME, &time);
    if (rc < 0) {
        LOG_ERR("time_init: Error while setting realtime clock. (%d)", rc);
        return rc;
    }

    LOG_INF("time_init: Realtime clock set to RTC time.");

    return 0;
}

SYS_INIT(time_init, POST_KERNEL, TIME_INIT_INIT_PRIORITY);