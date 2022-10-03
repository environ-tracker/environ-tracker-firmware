#include <zephyr.h>
#include <device.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>

LOG_MODULE_REGISTER(init);

#define LOG_INIT_INIT_PRIORITY  90


/**
 * @brief Set the FS logging backend filter to APP_FS_LOG_FILTER_LEVEL. This 
 *        means that only critical errors will be saved, saving FS space and 
 *        minimising wear.
 * 
 * @param dev UNUSED
 * @return 0 always
 */
int log_filter_init(const struct device *dev)
{
    const struct log_backend *backend;
    uint32_t set_level;

    backend = log_backend_get_by_name("log_backend_fs");
    if (backend == NULL) {
        return -1;
    }
    
    for (int i = 0; i < log_src_cnt_get(CONFIG_LOG_DOMAIN_ID); ++i) {
        set_level = log_filter_set(backend, CONFIG_LOG_DOMAIN_ID, i, 
                CONFIG_APP_FS_BACKEND_LOG_FILTER_LEVEL);

        if (set_level != CONFIG_APP_FS_BACKEND_LOG_FILTER_LEVEL) {
            LOG_ERR("log_init: Unable to set log level for source %d.", i);
        }
    }
    
    LOG_INF("log_init: FS backend set to "
            "CONFIG_APP_FS_BACKEND_LOG_FILTER_LEVEL");

    return 0;
}

SYS_INIT(log_filter_init, POST_KERNEL, LOG_INIT_INIT_PRIORITY);