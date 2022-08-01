cmake_minimum_required(VERSION 3.20.0)

set(CONF_FILE 
    prj.conf 
    log.conf 
    shell.conf 
    ble.conf
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(environ_tracker)

include_directories(inc)
target_sources(app 
    PRIVATE 
        src/shell.c 
        src/environ.c 
        src/imu.c 
        src/ble_localisation.c
)