zephyr_library()

zephyr_library_sources_ifdef(CONFIG_GPIO_SX1502 gpio_sx1502.c)
