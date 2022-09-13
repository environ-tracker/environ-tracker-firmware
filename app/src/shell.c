#include <zephyr.h>
#include <shell/shell.h>
#include <version.h>
#include <logging/log.h>
#include <stdlib.h>
#include <drivers/uart.h>
#include <drivers/gpio.h>

LOG_MODULE_REGISTER(app_shell);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static int cmd_led(const struct shell *shell, size_t argc, char **argv)
{
    static bool configured = false;
    static bool led_state = false;
    int ret = 0;

    if (!configured) {
        if (!device_is_ready(led.port)) {
            LOG_ERR("led1 device not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("led1 configure returned: %d", ret);
            return ret;
        }
        configured = true;
    }

    if (argv[1][1] == '\0') {
        switch (argv[1][0]) {
        case 'o':
        case 'f':
            led_state = (argv[1][0] == 'o') ? true : false;
            ret = gpio_pin_set_dt(&led, led_state);
            break;
        case 't':
            ret = gpio_pin_toggle_dt(&led);
            led_state = !led_state;
            break;
        case 'g':
            shell_print(shell, "led is %s", led_state ? "on" : "off");
            ret = 0;
            break;
        }
    } else {
        shell_print(shell, "Usage: led {o|f|t|g}");
        ret = -EINVAL;
    }

    return ret;
}

static int cmd_board(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(shell, CONFIG_BOARD);

    return 0;
}


SHELL_COND_CMD_ARG_REGISTER(DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay), led, 
        NULL, "led commands\n\r o : on\n\r f : off\n\r t : toggle\n\r "
        "g : get state", cmd_led, 2, 0);

SHELL_CMD_REGISTER(board, NULL, "Show board", cmd_board);