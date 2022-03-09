#define DT_DRV_COMPAT silabs_si1132

#include <drivers/sensor.h>
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

#include "si1132.h"

LOG_MODULE_REGISTER(si1132, CONFIG_SENSOR_LOG_LEVEL);

struct si1132_data {
    const struct device *i2c_dev;
    uint16_t vis_light;
    uint16_t ir_light;
    uint16_t uv_index;
    uint8_t calib[4];
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
        return -ENOTSUP;
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
 * @brief Writes to the Command register, performing the required Response 
 *        register checking
 * 
 * @param i2c_dev I2C device Si1132 device is on
 * @param cmd Command to write to the Command register
 * @return int 0 on success
 */
static int si1132_write_cmd_reg(const struct device *i2c_dev, uint8_t cmd)
{
    int rc;
    uint8_t response;

    /* Clear the Response register (send NOP command) */
    rc = i2c_reg_write_byte(i2c_dev, DT_INST_REG_ADDR(0), SI1132_REG_COMMAND,
            0x00);
    if (rc != 0) {
        LOG_ERR("Error while attempting to clear Response register");
        return rc;
    }

    /* Verify register is cleared */
    rc = i2c_reg_read_byte(i2c_dev, DT_INST_REG_ADDR(0), SI1132_REG_RESPONSE, 
            &response);
    if (rc != 0) {
        LOG_ERR("Error while attemping to verify Reponse register was cleared");
        return rc;
    }

    rc = si1132_response_handler(response);
    if (rc != 0) {
        return rc;
    }

    /* Write command into Command register */
    rc = i2c_reg_write_byte(i2c_dev, DT_INST_REG_ADDR(0), SI1132_REG_COMMAND,
            &cmd);
    if (rc != 0) {
        LOG_ERR("Error while attempting to write cmd: %02x to 
                Command register", cmd);
        return rc;
    }

    /* Verify Response register has changed */
    rc = i2c_reg_read_byte(i2c_dev, DT_INST_REG_ADDR(0), SI1132_REG_RESPONSE, 
            &response);
    if (rc != 0) {
        LOG_ERR("Error while attempting to verify Response register changed");
        return rc;
    }

    rc = si1132_response_handler(response);

    return rc;
}

// /**
//  * @brief 
//  * 
//  * @param i2c_dev I2C device Si1132 device is on
//  * @param si_data 
//  * @return int 0 on success
//  */
// static int si1132_get_light(const struct device *i2c_dev,
//                     struct si1132_data *si_data)
// {
//     int rc;
//     uint8_t data[4];

//     rc = i2c_burst_read(i2c_dev, DT_INST_REG_ADDR(0),
//             SI1132_REG_ALS_VIS_DATA, data, sizeof(data));
//     if (rc == 0) {
//         si_data->vis_light = sys_le16_to_cpu(data[0] << 8 | data[1]);
//         si_data->ir_light = sys_le_to_cpu(data[2] << 8 | data[3]);
//     } else {
//         LOG_ERR("Error while reading VIS and IR data registers");
//     }

//     return rc;
// }

// /**
//  * @brief 
//  * 
//  * @param i2c_dev I2C device Si1132 device is on
//  * @param si_data 
//  * @return int 0 on success
//  */
// static int si1132_get_uv(const struct device *i2c_dev,
//                     struct si1132_data *si_data)
// {
//     int rc;
//     uint8_t data[2];

//     rc = i2c_burst_read(i2c_dev, DT_INST_REG_ADDR(0),
//             SI1132_REG_AUX_DATA, sizeof(data));
//     if (rc == 0) {
//         si_data->uv_index = sys_le16_to_cpu(data[0] << 8 | data[1]);
//     } else {
//         LOG_ERR("Error while reading UV data register");
//     }

//     return rc;
// }

static int si1132_sample_fetch(const struct device *dev,
                    enum sensor_channel chan)
{
    int rc;
    uint8_t vis_ir_data[4], uv_data[2];
    struct si1132_data *si_data = dev->data;

    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

    /* Get visible and IR light data */
    rc = i2c_burst_read(i2c_dev, DT_INST_REG_ADDR(0),
            SI1132_REG_ALS_VIS_DATA, &vis_ir_data[0], sizeof(vis_ir_data));
    if (rc == 0) {
        si_data->vis_light = sys_le16_to_cpu(
                vis_ir_data[0] << 8 | vis_ir_data[1]);
        si_data->ir_light = sys_le_to_cpu(vis_ir_data[2] << 8 | vis_ir_data[3]);

        // TODO: apply factory calibration
    } else {
        LOG_ERR("Error while reading VIS and IR data registers");
        return rc;
    }

    /* Get UV data */
    rc = i2c_burst_read(i2c_dev, DT_INST_REG_ADDR(0),
            SI1132_REG_AUX_DATA,, &uv_data[0] sizeof(uv_data));
    if (rc == 0) {
        si_data->uv_index = sys_le16_to_cpu(uv_data[0] << 8 | uv_data[1]);
    } else {
        LOG_ERR("Error while reading UV data register");
        return rc;
    }

    /* Start the next conversion */
    rc = si1132_write_cmd_reg(si_data->i2c_dev, SI1132_CMD_ALS_FORCE);

    return rc;
}

static int si1132_channel_get(const struct device *dev,
                    enum sensor_channel chan,
                    struct sensor_value *val)
{
    struct si1132_data *si_data = dev->data;

    switch (chan) {
    SENSOR_CHAN_LIGHT:
        /* NOTE: Currently only 6lx resolution is supported */
        val->val1 = si_data->vis_data;
        val->val2 = 0;

        LOG_DBG("visible light (lx) = val1:%d, val2:%d", val->val1, val->val2);
        break;
    SENSOR_CHAN_IR:
        /* NOTE: Currently only 6lx resolution is supported */
        val->val1 = si_data->ir_data;
        val->val2 = 0;

        LOG_DBG("IR light (lx) = val1:%d, val2:%d", val->val1, val->val2);
        break;
    SENSOR_CHAN_RED:
        /* NOTE: This is actualy UV index */
        val->val1 = si_data->uv_data / 100;
        val->val2 = si_data->uv_data % 100 * 10000;

        LOG_DBG("UV index = val1:%d, val2:%d", val->val1, val->val2);
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api si1132_api = {
    .sample_fetch = &si1132_sample_fetch,
    .channel_get = &si1132_channel_get,
};

static int si1132_init(const struct device *dev)
{
    int rc;
    uint8_t ids[3] = {0};
    uint8_t meas_rate[2] = {0x00, 0x00};
    uint8_t ucoeff[4] = {0x7B, 0x6B, 0x01, 0x00};
    struct si1132_data *drv_data = dev->data;

    drv_data->i2c_dev = device_get_binding(DT_INST_BUS_LABEL(0));

    if (!drv_data->i2c_dev) {
        LOG_ERR("i2c master not found.");
        return -EINVAL;
    }

    /* Reset Si1132 */
    rc = i2c_reg_write_byte(drv_data->i2c_dev, DT_INST_REG_ADDR(0),
            SI1132_REG_COMMAND, SI1132_CMD_RESET);
    if (rc != 0) {
        return rc;
    }

    // TODO: wait 1ms for device to reset

    // Write to HW_KEY register as required
    rc = i2c_reg_write_byte(drv_data->i2c_dev, DT_INST_REG_ADDR(0),
            SI1132_REG_HW_KEY, SI1132_HW_KEY_MAGIC);
    if (rc != 0) {
        return rc;
    }

    rc = i2c_burst_read(drv_data->i2c_dev, DT_INST_REG_ADDR(0), 
            SI1132_REG_PART_ID, &ids[0], sizeof(ids));
    if (rc != 0) {
        return rc;
    }

    if (ids[0] == SI1132_PART_ID) {
        LOG_DBG("Si1132 device detected. PART_ID: %02x, REV_ID: %02x, 
                SEQ_ID: %02x", ids[0], ids[1], ids[2]);
    } else {
        LOG_ERR("Si1132 device not found");
        return -ENOTSUP;
    }

    /* 
     * Set the device in Forced Measurement mode but also handle the
     * error case where SEQ_ID = 0x01 which changes the location of
     * the MEAS_RATE registers 
     */
    if (ids[2] == 0x01) {
        rc = i2c_reg_write_byte(drv_data->i2c_dev, DT_INST_REG_ADDR(0),
                0x0A, meas_rate[0]);
        if (rc != 0) {
            return rc;
        }

        rc = i2c_reg_write_byte(drv_data->i2c_dev, DT_INST_REG_ADDR(0),
                0x08, meas_rate[1]);
        if (rc != 0) {
            return rc;
        }

    } else {
        rc = i2c_burst_write(drv_data->i2c_dev, DT_INST_REG_ADDR(0), 
                SI1132_REG_MEAS_RATE, &meas_rate[0], sizeof(meas_rate));
        if (rc != 0) {
            return rc;
        }
    }
    
    /* Enable visual, IR and UV conversions */
    rc = i2c_reg_write_byte(drv_data->i2c_dev, 
            DT_INST_REG_ADDR(0), SI1132_REG_PARAM_WR, 
            (SI1132_EN_ALS_VIS | SI1132_EN_ALS_IR | SI1132_EN_UV));
    if (rc != 0) {
        return rc;
    }

    rc = si1132_write_cmd_reg(drv_data->i2c_dev, 
            (SI1132_CMD_PARAM_SET | SI1132_PARAM_CHLIST));
    if (rc != 0) {
        return rc;
    }

    /* Write default coefficients */
    rc = i2c_burst_write(drv_data->i2c_dev, DT_INST_REG_ADDR(0),
            SI1132_REG_UCOEF, &ucoeff[0], sizeof(ucoeff));
    if (rc != 0) {
        return rc;
    }   

    /* Get factory calibration data */
    rc = si1132_write_cmd_reg(drv_data->i2c_dev, SI1132_CMD_GET_CAL);
    if (rc != 0) {
        return rc;
    }

    rc = i2c_burst_read(drv_data->i2c_dev, DT_INST_REG_ADDR(0),
            SI1132_REG_ALS_VIS_DATA, &drv_data->calib[0], 
            sizeof(drv_data->calib));
    if (rc != 0) {
        return rc;
    }

    /* Start the first conversion */
    rc = si1132_write_cmd_reg(drv_data->i2c_dev, SI1132_CMD_ALS_FORCE);
    if (rc != 0) {
        return rc;
    }

    LOG_DBG("si1132 init ok");

    return 0;
}

static struct si1132_data si_data;

DEVICE_DT_INST_DEFINE(0, si1132_init, NULL, &si_data, NULL,
        POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &si1132_api);