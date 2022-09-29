cmake_minimum_required(VERSION 3.20.0)

set(CONF_FILE 
    prj.conf 
    sensors.conf
    shell.conf 
    ble.conf
    lora.conf
)

set(BOARD environ_tracker2)
set(SHIELD environ_tracker_daughterboard_v2)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(environ_tracker)

include_directories(inc inc/file inc/ble)


target_sources(app 
    PRIVATE 
        src/shell.c 
        src/environ.c 
        src/imu.c 
        src/accumulator.c
        src/file/file_init.c
        src/file/file_common.c
        src/ble/ble_helpers.c
        src/ble/ble_network.c
        src/controller.c
        src/lorawan.c
        # src/ble_localisation.c
        tests/ble_tests.c
        src/init/gpio_init.c
        src/init/log_init.c
        src/init/time_init.c
)