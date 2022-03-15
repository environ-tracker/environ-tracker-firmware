cmake_minimum_required(VERSION 3.20.0)

set(CONF_FILE log.conf shell.conf)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(environ_tracker)

include_directories(inc)
target_sources(app PRIVATE src/shell.c)