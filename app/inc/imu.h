#ifndef IMU_H
#define IMU_H

#include <zephyr.h>
#include <drivers/sensor.h>

enum activity {
    ACTIVITY_UNDEFINED = 0,
    ACTIVITY_STAND,
    ACTIVITY_WALK,
    ACTIVITY_RUN,
    ACTIVITY_JUMP
};

struct imu_data {
    struct sensor_value accel_x;
    struct sensor_value accel_y;
    struct sensor_value accel_z;
    struct sensor_value gyro_x;
    struct sensor_value gyro_y;
    struct sensor_value gyro_z;
};

extern struct k_msgq activity_msgq;

#endif /* IMU_H */