#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>

#include "controller.h"

LOG_MODULE_DECLARE(init);


#define GPIO_INIT_INIT_PRIORITY 90

#define SW0_NODE    DT_ALIAS(sw0)
#define SW1_NODE    DT_ALIAS(sw1)
#define SW2_NODE    DT_ALIAS(sw2)


#define MIN_PULSE_TIME      10
#define LONG_PULSE_TIME     1000


enum button_map {
    BUTTON_LEFT = 0,
    BUTTON_RIGHT = 1,
    BUTTON_SELECT = 2,
    BUTTON_UNDEFINED = -1,
};


static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(SW2_NODE, gpios);

static struct gpio_callback button0_cb_data;
static struct gpio_callback button1_cb_data;
static struct gpio_callback button2_cb_data;


/**
 * @brief Maps the physical pin number of a button to its logical button number
 * 
 * @param pin Physcial pin number
 * @return Logical button number
 */
static enum button_map map_pin(uint32_t pin) 
{
    if (pin == BIT(button0.pin)) {
        return BUTTON_LEFT;
    } else if (pin == BIT(button1.pin)) {
        return BUTTON_SELECT;
    } else if (pin == BIT(button2.pin)) {
        return BUTTON_RIGHT;
    } else {
        return BUTTON_UNDEFINED;
    }
}

/**
 * @brief Callback for system buttons
 *
 * @param pins Bitfield of triggering pins
 */
static void button_pressed(const struct device *dev, struct gpio_callback *cb, 
        uint32_t pins)
{
    static const uint8_t pin_phy[3] = {button0.pin, button2.pin, button1.pin};
    static int64_t first_edge_time[3] = {0};

    int pin = map_pin(pins);

    if (pin == BUTTON_UNDEFINED) {
        LOG_WRN("Undefined button in button callback");
        return;
    }

    if (gpio_pin_get(dev, pin_phy[pin])) {
        LOG_INF("Button press first edge, pin: %d", pin);

        first_edge_time[pin] = k_uptime_get();
        return;
    } else {

        int64_t pulse_time = k_uptime_delta(&first_edge_time[pin]);
        
        LOG_INF("Button press second edge, pin: %d. pulse time: %lld", pin, pulse_time);
        
        if (pulse_time > MIN_PULSE_TIME) {

            k_event_post(&gpio_events, ((pulse_time < LONG_PULSE_TIME) ? 
                    SHORT_PRESS_EVENT : LONG_PRESS_EVENT) << pin);
        }
    }
}


/**
 * @brief Initialised a given GPIO as a double edge triggered interrupt source
 * 
 * @param gpio GPIO to initialise as interrupt source
 * @param callback Callback for the GPIO to call
 * @return 0 on success, else negative error code
 */
static int init_gpio(const struct gpio_dt_spec *gpio, 
        struct gpio_callback *callback)
{
    int ret;

    if (!device_is_ready(gpio->port)) {
        LOG_ERR("Button device %s is not ready", gpio->port->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(gpio, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Failed to configure GPIO on %s pin %d (%d)", 
                gpio->port->name, gpio->pin, ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(gpio, GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        LOG_ERR("Failed to configure interrupt on %s pin %d (%d)", 
                gpio->port->name, gpio->pin, ret);
        return ret;
    }

    gpio_init_callback(callback, button_pressed, BIT(gpio->pin));
    gpio_add_callback(gpio->port, callback);

    return 0;
}

/**
 * @brief Initialises GPIO
 * 
 * @return 0 on success, else negative error code
 */
int gpio_init(const struct device *dev)
{
    int rc;

    rc = init_gpio(&button0, &button0_cb_data);
    rc |= init_gpio(&button1, &button1_cb_data);
    rc |= init_gpio(&button2, &button2_cb_data);
    if (rc != 0) {
        LOG_ERR("Error while initialising buttons. (%d)", rc);
        return rc;
    }

    LOG_INF("Buttons initialised");

    return 0;
}

SYS_INIT(gpio_init, POST_KERNEL, GPIO_INIT_INIT_PRIORITY);