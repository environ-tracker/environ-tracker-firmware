#ifndef IMU_H
#define IMU_H

#include <drivers/sensor.h>

struct imu_data {
    struct sensor_value accel_x;
    struct sensor_value accel_y;
    struct sensor_value accel_z;
    struct sensor_value gyro_x;
    struct sensor_value gyro_y;
    struct sensor_value gyro_z;
};

#endif /* IMU_H */