#include <zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(controller);

#define CONTROLLER_STACK_SIZE  2048
#define CONTROLLER_PRIORITY    3

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
    LOG_ERR("Button pressed, pins: %d", pins);
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

void controller_thread(void *a, void *b, void *c)
{
    int ret;

    LOG_INF("Controller starting");

    ret = init_button(&button0, &button0_cb_data);
    ret |= init_button(&button1, &button1_cb_data);
    ret |= init_button(&button2, &button2_cb_data);
    if (ret != 0) {
        return;
    }

    LOG_INF("Controller started.");
}

K_THREAD_DEFINE(controller_id, CONTROLLER_STACK_SIZE, controller_thread, NULL, NULL, NULL, CONTROLLER_PRIORITY, 0, 0);