# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ibeacon)

target_sources(app PRIVATE src/main.c)

if(IBEACON_RSSI)
  zephyr_compile_definitions(IBEACON_RSSI=${IBEACON_RSSI})
endif()

if(IBEACON_MINOR)
  zephyr_compile_definitions(IBEACON_MINOR=${IBEACON_MINOR})
endif()

if(IBEACON_MAJOR)
  zephyr_compile_definitions(IBEACON_MAJOR=${IBEACON_MAJOR})
endif()

if (IBEACON_UUID_1)
  zephyr_compile_definitions(IBEACON_UUID_1=${IBEACON_UUID_1})
endif()

if (IBEACON_UUID_2)
  zephyr_compile_definitions(IBEACON_UUID_2=${IBEACON_UUID_2})
endif()

if (IBEACON_UUID_3)
  zephyr_compile_definitions(IBEACON_UUID_3=${IBEACON_UUID_3})
endif()

if (IBEACON_UUID_4)
  zephyr_compile_definitions(IBEACON_UUID_4=${IBEACON_UUID_4})
endif()

if (IBEACON_UUID_5)
  zephyr_compile_definitions(IBEACON_UUID_5=${IBEACON_UUID_5})
endif()