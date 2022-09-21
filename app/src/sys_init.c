#include <zephyr.h>
#include <device.h>
#include <posix/time.h>
#include <counter.h>

#include <logging/log_ctrl.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(init);

#if !DT_HAS_COMPAT_STATUS_OKAY(microcrystal_rv3028)
#error "Unsupported board: microcrystal,rv3028 not enabled on devicetree"
#endif

#define RTC_NODE DT_LABEL(DT_COMPAT_GET_ANY_STATUS_OKAY(microcrystal_rv3028))

/* Set LOG_LEVEL for FS backend to LOG_WRN */
#define FS_LOG_LEVEL 2

#define INIT_INIT_PRIORITY  95
#if INIT_INIT_PRIORITY < CONFIG_SENSOR_INIT_PRIORITY
#error "INIT_INIT_PRIORITY must be larger than SENSOR_INIT_PRIORITY"
#endif


void init(void)
{
    const struct device *rtc_dev = device_get_binding(RTC_NODE);
    struct timespec time = {0};
    time_t unix_time = 0;
    const struct log_backend *backend;
    uint32_t set_level;
    int rc;


    if (rtc_dev == NULL) {
        LOG_ERR("No RV3028 RTC device found");
        return;
    }

    rc = counter_get_value(rtc_dev, &unix_time);
    if (rc < 0) {
        LOG_ERR("Error while getting unix time from RTC (%d).", rc);
        return;
    }

    time.tv_sec = unix_time;

    rc = clock_settime(CLOCK_REALTIME, &time);
    if (rc < 0) {
        LOG_ERR("Error while setting realtime clock (%d).", rc);
        return;
    }

    LOG_DBG("Realtime clock set to RTC time");

    backend = log_backend_get_by_name("fs");
    
    for (int i = 0; i < log_src_cnt_get(CONFIG_LOG_DOMAIN_ID; ++i) {
        set_level = log_filter_set(backend, CONFIG_LOG_DOMAIN_ID, i, 
                FS_LOG_LEVEL);

        if (set_level != FS_LOG_LEVEL) {
            LOG_ERR("Unable to set log level for source %d.", i);
        }
    }
    
    LOG_INF("FS backend set to LOG_LEVEL_WRN");
}

SYS_INIT(init, POST_KERNEL, INIT_INIT_PRIORITY);