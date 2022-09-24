#include <kernel.h>
#include <device.h>
#include <init.h>
#include <string.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <drivers/i2c.h>
#include <stdio.h>
#include <stdlib.h>
#include <drivers/sensor.h>

#include "si1132.h"

LOG_MODULE_REGISTER(si1132, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT silabs_si1132


struct si1132_data {
    uint16_t vis_light;
    uint16_t ir_light;
    uint16_t uv_index;
    uint8_t calib[6];
};

struct si1132_config {
    struct i2c_dt_spec bus;
};


/**
 * @brief Handles the given response register value.
 * NOTE:  Currently just logs the errors
 * 
 * @param response Value to handle
 * @return int  0 if no error reported
 *             -ENOTSUP if invalid setting reported
 *             -EOVERFLOW if overflow reported 
 *             -ENOMSG if undocumented error reported
 */
static int si1132_response_handler(uint8_t response)
{
    switch (response) {
    case SI1132_RESPONSE_INVALID_SETTING:
        LOG_ERR("Invalid setting");
        return -EINVAL;
    case SI1132_RESPONSE_ALS_VIS_ADC_OVERFLOW:
        LOG_ERR("VIS ADC Overflow");
        break;
    case SI1132_RESPONSE_ALS_IR_ADC_OVERFLOW:
        LOG_ERR("IR ADC Overflow");
        break;
    case SI1132_RESPONSE_AUX_ADC_OVERFLOW:
        LOG_ERR("AUX ADC Overflow");
        break;
    default:
        if ((response & SI1132_RESPONSE_REG_MASK) != 0x00) {
            /* Undocumented error */
            LOG_ERR("Undocumented error code: %02x", response);
            return -ENOMSG;
        }
        return 0;
    }

    return -EOVERFLOW;
}

/**
 * @brief Writes a command to the command register, following the command
 *        protocol set in the Si1132 datasheet Section 4.2
 * 
 * @param dev Si1132 device to write to
 * @param cmd Command to write
 * @return 0 on success
 *        -EINVAL on an invalid setting error
 *        -EOVERFLOW on a register overflow error
 *        -ENOMSG on an undocumented error
 *         negative i2c error code on an  i2c error
 */
static int si1132_write_cmd_reg(const struct device *dev, uint8_t cmd)
{
    struct i2c_dt_spec *bus = &((struct si1132_config *)dev->config)->bus;
    uint8_t response, trys = 0;
    int err, handled_response;
    

    /* Clear the response register */
    err = i2c_reg_write_byte_dt(bus, SI1132_REG_COMMAND, SI1132_CMD_NOP);
    if (err != 0) {
        return err;
    }

    /* Verify response register is cleared */
    err = i2c_reg_read_byte_dt(bus, SI1132_REG_RESPONSE, &response);
    if (err != 0) {
        return err;
    } else if (response != SI1132_CMD_NOP) {
        return si1132_response_handler(response);
    }

    /* Run given command */
    err = i2c_reg_write_byte_dt(bus, SI1132_REG_COMMAND, cmd);
    if (err != 0) {
        return err;
    }

    /* Verify command ran without errors */
    while (1) {
        err = i2c_reg_read_byte_dt(bus, SI1132_REG_RESPONSE, &response);
        if (err != 0) {
            return err;
        }

        /* If no errors and incremented by 1, then command was successful */
        handled_response = si1132_response_handler(response);
        if (handled_response == 0 && response == 1) {
            return 0;
        }

        /* Sleep for 100us to wait */
        k_usleep(100);

        if (trys > SI1132_MAX_RESPONSE_REG_TRYS) {
            return handled_response;
        }
    };
}


static int si1132_sample_fetch(const struct device *dev, 
        enum sensor_channel chan)
{
    struct i2c_dt_spec *bus = &((struct si1132_config *)dev->config)->bus;
    struct si1132_data *si_data = dev->data;
    uint8_t vis_ir_data[4], uv_data[2];
    int err;
    

    /* Get visible and IR light data */
    err = i2c_burst_read_dt(bus, SI1132_REG_ALS_VIS_DATA, 
            vis_ir_data, ARRAY_SIZE(vis_ir_data));
    if (err != 0) {
        LOG_ERR("Error while reading VIS and IR data registers, err: %d", err);
        return err;
    }    

    // TODO: apply factory calibration
    si_data->vis_light = sys_le16_to_cpu(vis_ir_data[1] << 8 | vis_ir_data[0]);
    si_data->ir_light = sys_le16_to_cpu(vis_ir_data[3] << 8 | vis_ir_data[2]);


    /* Get UV data */
    err = i2c_burst_read_dt(bus, SI1132_REG_AUX_DATA, 
            uv_data, ARRAY_SIZE(uv_data));
    if (err != 0) {
        LOG_ERR("Error while reading UV data register, err: %d", err);
        return err;
    }

    si_data->uv_index = sys_le16_to_cpu(uv_data[1] << 8 | uv_data[0]);

  
    /* Start the next conversion */
    err = si1132_write_cmd_reg(dev, SI1132_CMD_ALS_FORCE);

    return err;
}

static int si1132_channel_get(const struct device *dev, 
        enum sensor_channel chan, struct sensor_value *val)
{
    struct si1132_data *si_data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_LIGHT:
        val->val1 = si_data->vis_light;
        val->val2 = 0;

        LOG_DBG("visible light (lx) = val1:%d, val2:%d", val->val1, val->val2);
        break;
    case SENSOR_CHAN_IR:
        val->val1 = si_data->ir_light;
        val->val2 = 0;

        LOG_DBG("IR light (lx) = val1:%d, val2:%d", val->val1, val->val2);
        break;
    case SENSOR_CHAN_RED:
        /* NOTE: This is actually UV index */
        val->val1 = si_data->uv_index / 100;
        val->val2 = si_data->uv_index % 100 * 10000;

        LOG_DBG("UV index = val1:%d, val2:%d", val->val1, val->val2);
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}



static int si1132_init(const struct device *dev)
{
    struct i2c_dt_spec *bus = &((struct si1132_config *)dev->config)->bus;
    struct si1132_data *data = dev->data;
    uint8_t ucoeff[4] = {0x7B, 0x6B, 0x01, 0x00};
    uint8_t ids[3] = {0};
    int err;


    LOG_DBG("Initialise device %s", dev->name);

    if (!device_is_ready(bus->bus)) {
        LOG_ERR("i2c bus not ready");
        return -ENODEV;
    }
    
    k_msleep(25);

    // Write to HW_KEY register as required
    err = i2c_reg_write_byte_dt(bus, SI1132_REG_HW_KEY, 
            SI1132_HW_KEY_MAGIC);
    if (err != 0) {
        return err;
    }

    err = i2c_burst_read_dt(bus, SI1132_REG_PART_ID, &ids[0], 
            ARRAY_SIZE(ids));
    if (err != 0) {
        return err;
    }

    if (ids[0] == SI1132_PART_ID) {
        LOG_DBG("Si1132 device detected. PART_ID: 0x%02x, REV_ID: 0x%02x, "
                "SEQ_ID: 0x%02x", ids[0], ids[1], ids[2]);
    } else {
        LOG_ERR("Si1132 device not found");
        return -ENOTSUP;
    }

    
    /* Enable visual, IR and UV conversions */
    err = i2c_reg_write_byte_dt(bus, SI1132_REG_PARAM_WR, 
            SI1132_EN_ALS_VIS | SI1132_EN_ALS_IR | SI1132_EN_UV);
    if (err != 0) {
        return err;
    }

    err = si1132_write_cmd_reg(dev, SI1132_CMD_PARAM_SET | SI1132_PARAM_CHLIST);
    if (err != 0) {
        return err;
    }


    /* Write default coefficients */
    err = i2c_burst_write_dt(bus, SI1132_REG_UCOEF, &ucoeff[0], 
            ARRAY_SIZE(ucoeff));
    if (err != 0) {
        return err;
    }   

    /* Get factory calibration data */
    err = si1132_write_cmd_reg(dev, SI1132_CMD_GET_CAL);
    if (err != 0) {
        return err;
    }

    err = i2c_burst_read_dt(bus, SI1132_REG_ALS_VIS_DATA, 
            &data->calib[0], 4);
    if (err != 0) {
        return err;
    }

    err = i2c_burst_read_dt(bus, SI1132_REG_AUX_DATA, 
            &data->calib[4], 2);
    if (err != 0) {
        return err;
    }


    /* Start the first conversion */
    err = si1132_write_cmd_reg(dev, SI1132_CMD_ALS_FORCE);
    if (err != 0) {
        return err;
    }

    LOG_DBG("Init ok");

    return 0;
}

static const struct sensor_driver_api si1132_api_funcs = {
    .sample_fetch = &si1132_sample_fetch,
    .channel_get = &si1132_channel_get,
};


#define SI1132_INIT(inst)                                       \
    static struct si1132_data si1132_data_##inst;               \
    static const struct si1132_config si1132_config_##inst = {  \
        .bus = I2C_DT_SPEC_INST_GET(inst),                      \
    };                                                          \
    DEVICE_DT_INST_DEFINE(inst,                                 \
            si1132_init,                                        \
            NULL,                                               \
            &si1132_data_##inst,                                \
            &si1132_config_##inst,                              \
            POST_KERNEL,                                        \
            CONFIG_SENSOR_INIT_PRIORITY,                        \
            &si1132_api_funcs)

DT_INST_FOREACH_STATUS_OKAY(SI1132_INIT);