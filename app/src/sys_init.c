#include <zephyr.h>
#include <device.h>
#include <posix/time.h>
#include <drivers/counter.h>
#include <drivers/gpio.h>

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


#define SW0_NODE    DT_ALIAS(sw0)
#define SW1_NODE    DT_ALIAS(sw1)
#define SW2_NODE    DT_ALIAS(sw2)

static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(SW2_NODE, gpios);

static struct gpio_callback button0_cb_data;
static struct gpio_callback button1_cb_data;
static struct gpio_callback button2_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb, 
        uint32_t pins)
{
    LOG_INF("Button pressed, pin: %d", pins);
}

static int init_button(const struct gpio_dt_spec *button, 
        struct gpio_callback *callback)
{
    int ret;

    if (!device_is_ready(button->port)) {
        LOG_ERR("Button device %s is not ready", button->port->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(button, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Failed to configure GPIO on %s pin %d (%d)", 
                button->port->name, button->pin, ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(button, GPIO_INT_EDGE_TO_INACTIVE);
    if (ret != 0) {
        LOG_ERR("Failed to configure interrupt on %s pin %d (%d)", 
                button->port->name, button->pin, ret);
        return ret;
    }

    gpio_init_callback(callback, button_pressed, BIT(button->pin));
    gpio_add_callback(button->port, callback);

    return 0;
}


int init(const struct device *dev)
{
    const struct device *rtc_dev = device_get_binding(RTC_NODE);
    struct timespec time = {0};
    time_t unix_time = 0;
    const struct log_backend *backend;
    uint32_t set_level;
    int rc;


    if (rtc_dev == NULL) {
        LOG_ERR("No RV3028 RTC device found");
        return -ENODEV;
    }

    rc = counter_get_value(rtc_dev, (uint32_t *)&unix_time);
    if (rc < 0) {
        LOG_ERR("Error while getting unix time from RTC (%d).", rc);
        return rc;
    }

    time.tv_sec = unix_time;

    rc = clock_settime(CLOCK_REALTIME, &time);
    if (rc < 0) {
        LOG_ERR("Error while setting realtime clock (%d).", rc);
        return rc;
    }

    LOG_INF("Realtime clock set to RTC time");

    backend = log_backend_get_by_name("log_backend_fs");
    
    for (int i = 0; i < log_src_cnt_get(CONFIG_LOG_DOMAIN_ID); ++i) {
        set_level = log_filter_set(backend, CONFIG_LOG_DOMAIN_ID, i, 
                FS_LOG_LEVEL);

        if (set_level != FS_LOG_LEVEL) {
            LOG_ERR("Unable to set log level for source %d.", i);
        }
    }
    
    LOG_INF("FS backend set to LOG_LEVEL_WRN");


    rc = init_button(&button0, &button0_cb_data);
    rc |= init_button(&button1, &button1_cb_data);
    rc |= init_button(&button2, &button2_cb_data);
    if (rc != 0) {
        LOG_ERR("Error while initialising buttons. (%d)", rc);
        return rc;
    }

    LOG_INF("Buttons initialised");

    return 0;
}

SYS_INIT(init, POST_KERNEL, INIT_INIT_PRIORITY);