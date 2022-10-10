#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(gui, CONFIG_LOG_DEFAULT_LEVEL);


#define GUI_THREAD_STACK_SIZE   1024
#define GUI_THREAD_PRIORITY     8


static int splash_screen_display(const struct device *dev);
void gui_thread(void *a, void *b, void *c);


K_THREAD_DEFINE(gui, GUI_THREAD_STACK_SIZE, gui_thread, NULL, NULL, NULL, 
        GUI_THREAD_PRIORITY, 0, 0);


void gui_thread(void *a, void *b, void *c)
{
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

    splash_screen_display(dev);

}

static int splash_screen_display(const struct device *dev)
{
    int rc;

    cfb_framebuffer_clear(dev, false);

    rc = cfb_print(dev, "0123456789", 0, 0);
    if (rc != 0) {
        LOG_ERR("Failed to write to CBF framebuffer. (%d)", rc);
        return rc;
    }

    cfb_framebuffer_finalize(dev);

    // k_sleep(K_SECONDS(5));

    // cfb_framebuffer_clear(dev, true);

    // k_sleep(K_SECONDS(1));

    return 0;
}