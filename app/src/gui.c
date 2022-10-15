#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/posix/time.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/display/cfb.h>
#include <zephyr/logging/log.h>

#include "accumulator.h"
#include "controller.h"
#include "environ.h"

LOG_MODULE_REGISTER(gui, CONFIG_LOG_DEFAULT_LEVEL);


#define GUI_THREAD_STACK_SIZE   1024
#define GUI_THREAD_PRIORITY     8


enum gui_screens {
    SCREEN_HOME = 0,
    SCREEN_ENVIRON,
    SCREEN_LOCATION,
    SCREEN_SETTINGS,
    NUM_SCREENS,
};


static int display_splash_screen(const struct device *dev);
static int display_home_screen(const struct device *dev);
static int display_environ_screen(const struct device *dev, 
        const struct environ_data *env_data);
static int display_location_screen(const struct device *dev, 
        const struct location *location, enum location_source source);
static int display_settings_screen(const struct device *dev);
static const char * get_screen_name(enum gui_screens screen);
void gui_thread(void *a, void *b, void *c);


K_THREAD_DEFINE(gui, GUI_THREAD_STACK_SIZE, gui_thread, NULL, NULL, NULL, 
        GUI_THREAD_PRIORITY, 0, 0);


void gui_thread(void *a, void *b, void *c)
{
    int rc;
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

    rc = display_splash_screen(dev);
    if (rc != 0) {
        LOG_ERR("Error (%d) while displaying splash screen", rc);
    }

    rc = display_home_screen(dev);
    if (rc != 0) {
        LOG_ERR("Error (%d) while displaying home screen", rc);
    }

    while (1) {

        /* Handle button controls */
        uint32_t events = k_event_wait(&gpio_events, BUTTON_SHORT_EVENTS_ALL, 
                false, K_MSEC(50));
        if (events) {
            if (events & LEFT_BUTTON_SHORT_EVENT) {
                if (current_screen == 0) {
                    /* Wrap the current screen around */
                    current_screen = NUM_SCREENS - 1;
                } else {
                    current_screen -= 1;
                }
            }
            if (events & RIGHT_BUTTON_SHORT_EVENT) {
                current_screen = (current_screen + 1) % NUM_SCREENS;
            }

            LOG_INF("screen is now: %s", get_screen_name(current_screen));

            k_event_set_masked(&gpio_events, 0, BUTTON_SHORT_EVENTS_ALL);
        }
        

        if (current_screen == SCREEN_SETTINGS) {
            display_settings_screen(dev);
        } else {
            rc = k_msgq_get(&gui_msgq, &sys_data, K_MSEC(50));
            if (rc != 0) {
                // continue;
            }

            switch (current_screen) {
            case SCREEN_HOME:
                rc = display_home_screen(dev);
                break;
            case SCREEN_ENVIRON:
                rc = display_environ_screen(dev, &sys_data.environ);
                break;
            case SCREEN_LOCATION:
                rc = display_location_screen(dev, &sys_data.location.location, 
                        sys_data.location.source);
                break;
            default:
                break;
            }

            if (rc != 0) {
                LOG_ERR("Error (%d) displaying %s screen", rc, 
                        get_screen_name(current_screen));
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
        return rc;
    }

    rc = cfb_print(dev, "Tracker", 28, 8 + 2*8);
    if (rc != 0) {
        return rc;
    }

    rc = cfb_print(dev, "v2.0", 44, 5*8);
    if (rc != 0) {
        return rc;
    }

    rc = cfb_framebuffer_finalize(dev);
    if (rc != 0) {
        return rc;
    }

    k_sleep(K_SECONDS(2));

    rc = cfb_framebuffer_clear(dev, true);
    if (rc != 0) {
        return rc;
    }

    k_sleep(K_MSEC(200));

    return 0;
}

static int display_home_screen(const struct device *dev)
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

        snprintk(buf, sizeof(buf), "BAT: %.1f%%", 
                (soc.val1 + (float)soc.val2 / 1000000));

        cfb_print(dev, buf, 0, 32);
    }


    cfb_framebuffer_finalize(dev);

    return 0;
}

static int display_environ_screen(const struct device *dev, 
        const struct environ_data *env_data)
{
    int rc;
    char buf[100];

    cfb_framebuffer_clear(dev, false);

    snprintk(buf, sizeof(buf), "%g C", 
            sensor_value_to_double(&env_data->temp));

    rc = cfb_print(dev, buf, 0, 0);
    if (rc != 0) {
        return rc;
    }

    snprintk(buf, sizeof(buf), "%g %%RH", 
            sensor_value_to_double(&env_data->humidity));

    rc = cfb_print(dev, buf, 0, 16);
    if (rc != 0) {
        return rc;
    }

    snprintk(buf, sizeof(buf), "%g kPa", 
            sensor_value_to_double(&env_data->press));

    rc = cfb_print(dev, buf, 0, 32);
    if (rc != 0) {
        return rc;
    }

    snprintk(buf, sizeof(buf), "UV Idx: %d", env_data->uv_index.val1);

    rc = cfb_print(dev, buf, 0, 48);
    if (rc != 0) {
        return rc;
    }

    cfb_framebuffer_finalize(dev);

    return 0;
}

static int display_location_screen(const struct device *dev, 
        const struct location *location, enum location_source source)
{
    int rc;
    char buf[100];
    
    cfb_framebuffer_clear(dev, false);

    snprintk(buf, sizeof(buf), "LON %f", location->longitude);

    rc = cfb_print(dev, buf, 0, 0);

    snprintk(buf, sizeof(buf), "LAT %f", location->latitude);

    rc = cfb_print(dev, buf, 0, 16);

    snprintk(buf, sizeof(buf), "ALT %.1f", location->altitude);

    rc = cfb_print(dev, buf, 0, 32);

    snprintk(buf, sizeof(buf), "Source: %s", 
            (source == LOCATION_BLE) ? "BLE" : "GPS");

    rc = cfb_print(dev, buf, 0, 48);
    
    cfb_framebuffer_finalize(dev);

    return 0;
}

static int display_settings_screen(const struct device *dev)
{
    int rc;
    
    cfb_framebuffer_clear(dev, false);

    rc = cfb_print(dev, "Settings", 16, 16);

    cfb_framebuffer_finalize(dev);

    return rc;
}

static const char * get_screen_name(enum gui_screens screen)
{
    static const char *names[] = {"home", "environ", "location", "settings"};

    if (screen < 0 || screen >= NUM_SCREENS) {
        return "error";
    }

    return names[screen];
}