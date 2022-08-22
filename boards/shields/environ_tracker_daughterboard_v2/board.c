#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(daughter_pwr_ctrl);

#define DAUGHTER_PWR_CTRL_GPIO_PIN 4

struct pwr_ctrl_cfg {
    const char *port;
    uint32_t pin;
};

static int pwr_ctrl_init(const struct device *dev) 
{
    const struct pwr_ctrl_cfg *cfg = dev->config;
    const struct device *gpio;

    gpio = device_get_binding(cfg->port);
    if (!gpio) {
        printk("Could not bind device \"%s\"\n", cfg->port);
        return -ENODEV;
    }

    gpio_pin_configure(gpio, cfg->pin, GPIO_OUTPUT_HIGH);

    k_sleep(K_MSEC(1));
    
    return 0;
}

#if CONFIG_BOARD_DAUGHTER_PWR_CTRL_INIT_PRIORITY <= CONFIG_GPIO_INIT_PRIORITY
#error GPIO_INIT_PRIORITY must be lower than BOARD_DAUGHTER_PWR_CTRL_INIT_PRIORITY
#endif

static const struct pwr_ctrl_cfg daughter_pwr_ctrl_cfg = {
    .port = DT_LABEL(DT_NODELABEL(sx1508b)),
    .pin = DAUGHTER_PWR_CTRL_GPIO_PIN,
};

DEVICE_DEFINE(daughter_pwr_ctrl_init, "", pwr_ctrl_init, NULL, NULL, 
        &daughter_pwr_ctrl_cfg, POST_KERNEL, 
        CONFIG_BOARD_DAUGHTER_PWR_CTRL_INIT_PRIORITY, NULL);