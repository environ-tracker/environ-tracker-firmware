#include <stdio.h>
#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <sys/ring_buffer.h>
#include <logging/log.h>

#include "imu.h"

LOG_MODULE_REGISTER(imu);


#define IMU_NODE DT_LABEL(DT_COMPAT_GET_ANY_STATUS_OKAY(st_lsm6dso))

#if !DT_HAS_COMPAT_STATUS_OKAY(st_lsm6dso)
#error "Unsupported board: st,lsm6dso not enabled on devicetree"
#endif

#define IMU_STACK_SIZE  500
#define IMU_PRIORITY    4

#define IMU_DATA_WORD_SIZE  sizeof(struct imu_data)/sizeof(uint32_t)

#define IMU_ODR 52
#define IMU_RINGBUF_SIZE    IMU_ODR * IMU_DATA_WORD_SIZE * 5


RING_BUF_ITEM_DECLARE_SIZE(imu_ring_buf, IMU_RINGBUF_SIZE);


static void data_trig_handler(const struct device *dev, 
        const struct sensor_trigger *trig)
{
    struct imu_data data;
    static int trig_counter = 0;
    int ret;

    trig_counter++;

    sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
    sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);

    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &data.accel_x);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &data.accel_y);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &data.accel_z);

    sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &data.gyro_x);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &data.gyro_y);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &data.gyro_z);

    LOG_DBG("accel x:%d.%06d, y:%d.%06d, z:%d.%06d", data.accel_x.val1, 
            data.accel_x.val2, data.accel_y.val1, data.accel_y.val2, 
            data.accel_z.val1, data.accel_z.val2);

    LOG_DBG("gyro x:%d.%06d, y:%d.%06d, z:%d.%06d", data.gyro_x.val1, 
            data.gyro_x.val2, data.gyro_y.val1, data.gyro_y.val2, 
            data.gyro_z.val1, data.gyro_z.val2);

    LOG_DBG("trig_counter: %d", trig_counter);

    ret = ring_buf_item_put(&imu_ring_buf, 0, 0, (uint32_t *)&data,  
            IMU_DATA_WORD_SIZE);
    if (ret != 0) {
        // NOTE not enough space, partial copy, shouldn't happen
        LOG_ERR("Partial copy to imu_ring_buf, %d", ret);
        ring_buf_reset(&imu_ring_buf);
    }
}

static void tap_trig_handler(const struct device *dev, 
        const struct sensor_trigger *trig)
{
    
}

void imu_thread(void *a, void *b, void *c)
{
    struct sensor_trigger data_trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ACCEL_XYZ | SENSOR_CHAN_GYRO_XYZ
    };

    struct sensor_trigger tap_trig = {
        .type = SENSOR_TRIG_TAP,
        .chan = SENSOR_CHAN_ACCEL_XYZ
    };

    struct sensor_value odr_attr = {
        .val1 = IMU_ODR,
        .val2 = 0
    };
    
    const struct device *imu_dev = device_get_binding(IMU_NODE);
    if (imu_dev == NULL) {
        LOG_ERR("No device %s found.", IMU_NODE);
        return;
    }

    LOG_DBG("Device %p name is %s", imu_dev, imu_dev->name);

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, 
            SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) != 0) {
        LOG_ERR("Couldn't set accel ODR to %dHZ", odr_attr.val1);
        return;
    }

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, 
            SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) != 0) {
        LOG_ERR("Couldn't set gyro ODR to %dHZ", odr_attr.val1);
        return;
    }

    if (sensor_trigger_set(imu_dev, &data_trig, data_trig_handler) != 0) {
        LOG_ERR("Coundn't set data ready trigger");
        return;
    }

    if (sensor_trigger_set(imu_dev, &tap_trig, tap_trig_handler) != 0) {
        LOG_ERR("Coundn't set data ready trigger");
        return;
    }

    struct imu_data data[IMU_ODR];

    while (1) {
        
        // NOTE: Potentially change this to a semaphore given from ISR, where ISR checks ring_buf_size?
        if ((ring_buf_size_get(&imu_ring_buf) / IMU_DATA_WORD_SIZE) > IMU_ODR) {
            // LOG_INF("There is enough data items");
            ring_buf_get_claim(&imu_ring_buf, (uint8_t **)&data, 
                    IMU_ODR * IMU_DATA_WORD_SIZE);

            // TODO: process claimed data

            ring_buf_get_finish(&imu_ring_buf, IMU_ODR * IMU_DATA_WORD_SIZE);
        } else {
            k_msleep(20);
        }

        
    }
}

K_THREAD_DEFINE(imu_id, IMU_STACK_SIZE, imu_thread, NULL, NULL, NULL, 
        IMU_PRIORITY, 0, 0);