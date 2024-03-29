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

set(NANOPB_OPTIONS "-I${CMAKE_SOURCE_DIR}")
nanopb_generate_cpp(proto_sources proto_headers RELPATH .
    src/network/proto/upload_data.proto
)

zephyr_library_include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(inc inc/file inc/ble inc/network inc/gnss inc/gnss/ubx)


target_sources(app 
    PRIVATE 
        ${proto_sources}
        src/ble/ble_localisation.c
        src/shell.c 
        src/controller.c
        src/accumulator.c
        src/environ.c 
        src/imu.c 
        src/init/gpio_init.c
        src/init/log_init.c
        src/init/time_init.c
        src/file/file_init.c
        src/file/file_common.c
        src/network/lorawan.c
        src/network/encode_pb.c
        src/ble/ble_network.c
        src/gnss/gnss.c
        src/gnss/gnss_shell_cmds.c
        src/gnss/minmea.c
        src/gnss/ubx/u_ubx_protocol.c
        src/gui.c
)