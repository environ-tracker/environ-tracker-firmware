#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "gnss.h"


static int cmd_toggle_gnss_debug_output(const struct shell *shell, 
        size_t argc, char ** argv)
{
    static bool debug_enabled = false;

    k_event_post(&gnss_control_events, GNSS_TOGGLE_DEBUG_OUTPUT);
    debug_enabled = !debug_enabled;

    shell_print(shell, "GNSS debug output toggled. Current state: %s", 
            (debug_enabled) ? "on" : "off");

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(gnss_cmds, 
        SHELL_CMD(toggle_debug, NULL, "toggle debug NMEA output", 
                cmd_toggle_gnss_debug_output),
        SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(gnss, &gnss_cmds, "GNSS commands", NULL);