#ifndef ACCUMULATOR_H
#define ACCUMULATOR_H

#include <zephyr/kernel.h>

#include "environ.h"
// #include "ble_network.h"
#include "imu.h"
#include "location.h"


#define ENVIRON_DATA_PENDING    1 << 0
#define ACTIVITY_DATA_PENDING   1 << 1
#define LOCATION_DATA_PENDING   1 << 2


struct system_data {
    uint32_t timestamp;
    struct environ_data environ;
    struct location_wrapper location;
    enum activity activity;
};

extern struct k_msgq lorawan_msgq;

extern struct k_event data_events;

#endif /* ACCUMULATOR_H */