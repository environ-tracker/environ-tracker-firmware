#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#include "environ.h"

LOG_MODULE_REGISTER(environ_fetch);


/* Check if the board has the required sensors */
#if !DT_HAS_COMPAT_STATUS_OKAY(bosch_bme680)
#error "Unsupported board: bosch,bme680 not enabled on devicetree"
#endif

#if !DT_HAS_COMPAT_STATUS_OKAY(silabs_si1132)
#error "Unsupported board: silabs,si1132 not enabled on devicetree"
#endif


#define BME680_NODE DT_LABEL(DT_COMPAT_GET_ANY_STATUS_OKAY(bosch_bme680))
#define SI1132_NODE DT_LABEL(DT_COMPAT_GET_ANY_STATUS_OKAY(silabs_si1132))

#define ENVIRON_STACK_SIZE  500
#define ENVIRON_PRIORITY    5

#define ENVIRON_DATA_PERIOD 3000


K_MSGQ_DEFINE(environ_data_msgq, sizeof(struct environ_data), 10, 4);


/**
 * @brief Fetches all environmental sensor data and sends onwards
 */
void environ_fetch_thread(void *a, void *b, void *c)
{
    struct environ_data data = {0};

    const struct device *bme680_dev = device_get_binding(BME680_NODE);
    const struct device *si1132_dev = device_get_binding(SI1132_NODE);

    if (bme680_dev == NULL || si1132_dev == NULL) {
        LOG_ERR("No %s device found", (bme680_dev) ? "Si1132" : "BME680");
    }

    LOG_DBG("Device %p name is %s", bme680_dev, bme680_dev->name);
    LOG_DBG("Device %p name is %s", si1132_dev, si1132_dev->name);


    while (1) {
        k_msleep(ENVIRON_DATA_PERIOD);
    
        /* Fetch all sensor channels */
        sensor_sample_fetch(bme680_dev);
        sensor_sample_fetch(si1132_dev);

        /* Get all channel values */
        sensor_channel_get(bme680_dev, SENSOR_CHAN_AMBIENT_TEMP, &data.temp);
        sensor_channel_get(bme680_dev, SENSOR_CHAN_PRESS, &data.press);
        sensor_channel_get(bme680_dev, SENSOR_CHAN_HUMIDITY, &data.humidity);
        sensor_channel_get(bme680_dev, SENSOR_CHAN_GAS_RES, &data.gas_res);
        
        sensor_channel_get(si1132_dev, SENSOR_CHAN_LIGHT, &data.vis_light);
        sensor_channel_get(si1132_dev, SENSOR_CHAN_IR, &data.ir_light);
        sensor_channel_get(si1132_dev, SENSOR_CHAN_RED, &data.uv_index);

        LOG_DBG("T: %d.%06d; P: %d.%06d; H: %d.%06d; G: %d.%06d; "
                "L: %d; IR: %d; UV: %d.%06d", data.temp.val1, 
                data.temp.val2, data.press.val1, data.press.val2, 
                data.humidity.val1, data.humidity.val2, data.gas_res.val1, 
                data.gas_res.val2, data.vis_light.val1, data.ir_light.val1,
                data.uv_index.val1, data.uv_index.val2);

        /* Send environmental data */
        while (k_msgq_put(&environ_data_msgq, &data, K_NO_WAIT) != 0) {
            /* message queue is full: purge old data & try again */
            k_msgq_purge(&environ_data_msgq);
            LOG_DBG("environ_data_msgq has been purged");
        }

    }
}

K_THREAD_DEFINE(environ_id, ENVIRON_STACK_SIZE, environ_fetch_thread, 
        NULL, NULL, NULL, ENVIRON_PRIORITY, 0, 0);