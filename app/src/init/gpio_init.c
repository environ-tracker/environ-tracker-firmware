#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>

#include <logging/log.h>

LOG_MODULE_DECLARE(init);


#define GPIO_INIT_INIT_PRIORITY 90

#define SW0_NODE    DT_ALIAS(sw0)
#define SW1_NODE    DT_ALIAS(sw1)
#define SW2_NODE    DT_ALIAS(sw2)


static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(SW2_NODE, gpios);

static struct gpio_callback button0_cb_data;
static struct gpio_callback button1_cb_data;
static struct gpio_callback button2_cb_data;


static void button_pressed(const struct device *dev, struct gpio_callback *cb, 
        uint32_t pins)
{
    uint8_t pin = -1;

    if (pins == BIT(button0.pin)) {
        pin = button0.pin;
    } else if (pins == BIT(button1.pin)) {
        pin = button1.pin;
    } else if (pins == BIT(button2.pin)) {
        pin = button2.pin;
    }

    if (gpio_pin_get(dev, pin)) {
        LOG_INF("Button press first edge, pin: %d", pin);
    } else {
        LOG_INF("Button press second edge, pin: %d", pin);
    }
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

    ret = gpio_pin_interrupt_configure_dt(button, GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        LOG_ERR("Failed to configure interrupt on %s pin %d (%d)", 
                button->port->name, button->pin, ret);
        return ret;
    }

    gpio_init_callback(callback, button_pressed, BIT(button->pin));
    gpio_add_callback(button->port, callback);

    return 0;
}

int gpio_init(const struct device *dev)
{
    int rc;

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

SYS_INIT(gpio_init, POST_KERNEL, GPIO_INIT_INIT_PRIORITY);