#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/posix/time.h>
#include <zephyr/drivers/counter.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(init);


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
    struct timespec time = {0};
    time_t unix_time = 0;
    int rc;

    const struct device *rtc_dev = DEVICE_DT_GET_ONE(microcrystal_rv3028);

    if (!device_is_ready(rtc_dev)) {
        LOG_ERR("time_init: %s device not ready.", rtc_dev->name);
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