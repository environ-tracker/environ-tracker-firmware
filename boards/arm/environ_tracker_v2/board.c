#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define MOTHER_VDD_PWR_CTRL_GPIO_PIN 19

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

#if CONFIG_BOARD_MOTHER_VDD_PWR_CTRL_INIT_PRIORITY <= CONFIG_GPIO_INIT_PRIORITY
#error GPIO_INIT_PRIORITY must be lower than BOARD_VDD_PWR_CTRL_INIT_PRIORITY
#endif

static const struct pwr_ctrl_cfg mother_vdd_pwr_ctrl_cfg = {
    .port = DT_LABEL(DT_NODELABEL(gpio0)),
    .pin = MOTHER_VDD_PWR_CTRL_GPIO_PIN,
};

DEVICE_DEFINE(mother_vdd_pwr_ctrl_init, "", pwr_ctrl_init, NULL, NULL, 
        &mother_vdd_pwr_ctrl_cfg, POST_KERNEL, 
        CONFIG_BOARD_VDD_PWR_CTRL_INIT_PRIORITY, NULL);