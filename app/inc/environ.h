#ifndef ENVIRON_H
#define ENVIRON_H

#include <zephyr.h>
#include <drivers/sensor.h>

struct environ_data {
    struct sensor_value temp;
    struct sensor_value press;
    struct sensor_value humidity;
    struct sensor_value gas_res;
    struct sensor_value vis_light;
    struct sensor_value ir_light;
    struct sensor_value uv_index;
}__attribute__((aligned(4)));

extern struct k_msgq environ_data_msgq;

// extern const k_id_t environ_id; // NOTE: If thread reference required, uncomment

#endif /* ENVIRON_H */