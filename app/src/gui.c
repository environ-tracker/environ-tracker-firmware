#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/posix/time.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/display/cfb.h>
#include <zephyr/logging/log.h>

#include "accumulator.h"
#include "controller.h"

LOG_MODULE_REGISTER(gui, CONFIG_LOG_DEFAULT_LEVEL);


#define GUI_THREAD_STACK_SIZE   1024
#define GUI_THREAD_PRIORITY     8


static int display_splash_screen(const struct device *dev);
static int display_home_screen(const struct device *dev, 
        const struct system_data *sys_data);
static int display_environ_screen(const struct device *dev, 
        const struct system_data *sys_data);
static int display_location_screen(const struct device *dev, 
        const struct system_data *sys_data);
void gui_thread(void *a, void *b, void *c);


enum gui_screens {
    SCREEN_HOME = 0,
    SCREEN_ENVIRON,
    SCREEN_LOCATION,
    SCREEN_SETTINGS,
    NUM_SCREENS,
};



K_THREAD_DEFINE(gui, GUI_THREAD_STACK_SIZE, gui_thread, NULL, NULL, NULL, 
        GUI_THREAD_PRIORITY, 0, 0);


void gui_thread(void *a, void *b, void *c)
{
    int ret;
    struct system_data sys_data = {0};
    enum gui_screens current_screen = SCREEN_HOME;

    LOG_INF("GUI started.");

    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));

    if (!device_is_ready(dev)) {
        LOG_ERR("Display device not ready.");
        return;
    }

    if (display_set_pixel_format(dev, PIXEL_FORMAT_MONO10) != 0) {
        LOG_ERR("Failed to set pixel format.");
        return;
    }

    if (cfb_framebuffer_init(dev) != 0) {
        LOG_ERR("Failed to initialise CBF framebuffer.");
        return;
    }

    cfb_framebuffer_clear(dev, true);

    display_blanking_off(dev);

    cfb_framebuffer_set_font(dev, 0);

    LOG_INF("x_res %d, y_res %d, ppt %d, rows %d, cols %d", 
            cfb_get_display_parameter(dev, CFB_DISPLAY_WIDTH), 
            cfb_get_display_parameter(dev, CFB_DISPLAY_HEIGH),
            cfb_get_display_parameter(dev, CFB_DISPLAY_PPT),
            cfb_get_display_parameter(dev, CFB_DISPLAY_ROWS),
            cfb_get_display_parameter(dev, CFB_DISPLAY_COLS));

    display_splash_screen(dev);

    display_home_screen(dev, &sys_data);

    while (1) {

        /* Handle button controls */
        uint32_t events = k_event_wait(&gpio_events, BUTTON_SHORT_EVENTS_ALL, false, K_MSEC(50));
        if (events) {
            if (events & LEFT_BUTTON_SHORT_EVENT) {
                current_screen -= 1;
                if (current_screen < 0) {
                    current_screen = SCREEN_SETTINGS;
                }
            }
            if (events & RIGHT_BUTTON_SHORT_EVENT) {
                current_screen = (current_screen + 1) % NUM_SCREENS;
            }

            k_event_set_masked(&gpio_events, 0, BUTTON_SHORT_EVENTS_ALL);
        }
        

        if (current_screen == SCREEN_SETTINGS) {
            LOG_WRN("Settings screen not implemented");
        } else {
            // ret = k_msgq_get(&gui_msgq, &sys_data, K_MSEC(50));
            // if (ret) {
            //     continue;
            // }

            switch (current_screen) {
            case SCREEN_HOME:
                display_home_screen(dev, &sys_data);
                break;
            case SCREEN_ENVIRON:
                display_environ_screen(dev, &sys_data);
                break;
            case SCREEN_LOCATION:
                display_location_screen(dev, &sys_data);
                break;
            default:
                break;
            }
        }
        
    }

}

static int display_splash_screen(const struct device *dev)
{
    int rc;

    cfb_framebuffer_clear(dev, false);

    rc = cfb_print(dev, "Environ", 28, 8);
    if (rc != 0) {
        LOG_ERR("Failed to write to CBF framebuffer. (%d)", rc);
        return rc;
    }

    rc = cfb_print(dev, "Tracker", 28, 8 + 2*8);
    if (rc != 0) {
        LOG_ERR("Failed to write to CBF framebuffer. (%d)", rc);
        return rc;
    }

    rc = cfb_print(dev, "v2.0", 44, 5*8);
    if (rc != 0) {
        LOG_ERR("Failed to write to CBF framebuffer. (%d)", rc);
        return rc;
    }

    cfb_framebuffer_finalize(dev);

    k_sleep(K_SECONDS(2));

    cfb_framebuffer_clear(dev, true);

    k_sleep(K_MSEC(200));

    return 0;
}

static int display_home_screen(const struct device *dev, 
        const struct system_data *sys_data)
{
    const struct device *gauge_dev = DEVICE_DT_GET(DT_NODELABEL(lc709204f));
    struct timespec time;
    struct tm tm;
    bool gauge_ok = true;
    char buf[100];

    if (!device_is_ready(gauge_dev)) {
        LOG_ERR("Fuel Gauge device is not ready");
        gauge_ok = false;
    }

    clock_gettime(CLOCK_REALTIME, &time);
    gmtime_r(&time.tv_sec, &tm);

    cfb_framebuffer_clear(dev, false);

    snprintk(buf, sizeof(buf), "%d-%02u-%02u",  tm.tm_year + 1900, 
            tm.tm_mon + 1, tm.tm_mday);

    cfb_print(dev, buf, 0, 0);

    snprintk(buf, sizeof(buf), "%02u:%02u:%02u UTC", tm.tm_hour, tm.tm_min, 
            tm.tm_sec);

    cfb_print(dev, buf, 0, 16);


    if (gauge_ok) {
        struct sensor_value soc; 

        sensor_sample_fetch(gauge_dev);
        sensor_channel_get(gauge_dev, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &soc);

        snprintk(buf, sizeof(buf), "BAT: %.f%%", 
                (soc.val1 + (float)soc.val2 / 1000000));

        cfb_print(dev, buf, 0, 32);
    }


    cfb_framebuffer_finalize(dev);

    return 0;
}

static int display_environ_screen(const struct device *dev, 
        const struct system_data *sys_data)
{
    return 0;
}

static int display_location_screen(const struct device *dev, 
        const struct system_data *sys_data)
{
    return 0;
}