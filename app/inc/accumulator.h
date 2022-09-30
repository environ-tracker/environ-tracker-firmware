#ifndef ACCUMULATOR_H
#define ACCUMULATOR_H

#include <zephyr.h>

#include "environ.h"
#include "ble_network.h"
#include "imu.h"

struct system_data {
    uint32_t timestamp;
    struct environ_data environ;
    struct location location;
    enum activity activity;
}__attribute__((aligned(4)));

extern struct k_msgq lorawan_msgq;

#endif /* ACCUMULATOR_H */