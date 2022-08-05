// #include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <sys/byteorder.h>
#include <sys/crc.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(lc709204f, CONFIG_SENSOR_LOG_LEVEL);

#include "lc709204f.h"

#define DT_DRV_COMPAT onsemi_lc709204f


/**
 * @brief Write to a 16b register
 * 
 * @param spec The I2C bus devicetree spec
 * @param reg_addr Register address to write to
 * @param value Value to write to register
 * @return 0 if successful, or negative error code from I2C API
 */
static int lc709204f_reg_write(const struct i2c_dt_spec *spec, 
        uint8_t reg_addr, uint16_t value) 
{
    // Addresses added to make CRC-8 calculation easier
    uint8_t buf[5] = {spec->addr, reg_addr, value, value >> 8, 0};

    // Calculate CRC-8
    buf[4] = crc8(buf, 4, 0x07, 0, false);

    return i2c_write_dt(spec, &buf[1], 4);
}

/**
 * @brief Read a 16b register value
 * NOTE: Currently the CRC-8 value is not checked
 * 
 * @param spec The I2C bus devicetree spec
 * @param reg_addr Register address to read
 * @param value Place to put the value on success
 * @return 0 if successful, or negative error code from I2C API
 */
static int lc709204f_16b_reg_read(const struct i2c_dt_spec *spec,
        uint8_t reg_addr, uint16_t *value)
{
    uint8_t buf[3] = {0};

    // Read 2 data bytes and crc byte
    int err = i2c_burst_read_dt(spec, reg_addr, buf, 3);
    if (err != 0) {
        return err;
    }

    // Save register data in sys endianness
    *value = (buf[1] << 8) | buf[0];
    *value = le16_to_sys_cpu(*value);

    return 0;
}

/**
 * @brief Read a 32b register value
 * NOTE: Currently teh CRC-8 value is not checked
 * 
 * @param spec The I2C bus devicetree spec
 * @param reg_addr Register address to read
 * @param value Place to put the value on success
 * @return 0 if successful, or negative error code from I2C API
 */
static int lc709204f_32b_reg_read(const struct i2c_dt_spec *spec,
        uint8_t reg_addr, uint32_t *value)
{
    // Addresses added to make CRC-8 check easier
    uint8_t buf[5] = {0};

    int err = i2c_burst_read_dt(spec, reg_addr, buf, 5);
    if (err != 0) {
        return err;
    }

    // Save register data in sys endianness
    *value = (buf[3] << (3 * 8)) | (buf[2] << (2 * 8)) | (buf[1] << 8) | buf[0];
    *value = le32_to_sys_cpu(*value);
    return 0;
}

/**
 * @brief Converts the LC709204F's temperature representation to units of 0.1 
 *        degrees celsius
 * 
 * @param lc_temperature LC709204F temperature value
 * @return The LC709204F temperature value in 0.1 degrees celsius 
 */
static int convert_lc709204f_temperature_to_celsius(uint16_t lc_temperature)
{
    return lc_temperature - LC709204F_0C_VALUE;
}

static int lc709204f_sample_fetch(const struct device *dev, 
        enum sensor_channel chan) 
{

}

// TODO: check val2's 
static int lc709204f_channel_get(const struct device *dev, 
        enum sensor_channel chan, struct sensor_value *val)
{
    struct battery_data specs = dev->config->battery_specs;
    struct lc709204f_data data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_GAUGE_TEMP:
        val->val1 = data.temp;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        val->val1 = data.state_of_charge / 10;
        val->val2 = data.state_of_charge % 10;
        break;
    case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
        val->val1 = specs.capacity;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
        double remain_capacity = specs.capacity * data.state_of_charge / 1000;

        val->val1 = (uint16_t)remain_capacity;
        val->val2 = (uint16_t)((remain_capacity % 1000) * 1000);
        break;
    case SENSOR_CHAN_GAUGE_STATE_OF_HEALTH:
        val->val1 = data.state_of_health;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_GAUGE_TIME_TO_EMPTY:
        val->val1 = data.time_to_empty;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_GAUGE_TIME_TO_FULL:
        val->val1 = data.time_to_full;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_GAUGE_CYCLE_COUNT:
        val->val1 = data.cycle_count;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE:
        val->val1 = specs.max_voltage / 1000;
        val->val2 = specs.max_voltage % 1000;
        break;
    case SENSOR_CHAN_GAUGE_DESIRED_VOLTAGE:
        val->val1 = specs.nom_voltage / 1000;
        val->val2 = specs.nom_voltage % 1000;
        break;
    case SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT:
        val->val1 = specs.charge_current / 1000;
        val->val2 = specs.charge_current % 1000;
        break;
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        val->val1 = data.voltage / 1000;
        val->val2 = data.voltageq % 1000;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}



// TODO: CHange to macros
static int lc709204f_convert_battery_type_to_param(int battery_type)
{
    switch (battery_type) {
    case 1:
        return LC709204F_BATTERY_01;
    case 4:
        return LC709204F_BATTERY_04;
    case 5:
        return LC709204F_BATTERY_05;
    case 6:
        return LC709204F_BATTERY_06;
    case 7:
        return LC709204F_BATTERY_07;
    default:
        return -EINVAL;
    }
}

// TODO: add overflow checks for config vals
static int lc709204f_init(const struct device *dev)
{
    struct lc709204f_data *data = dev->data;
    struct lc709204f_config *config = dev->config;
    int err;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("i2c bus is not ready");
        return -ENODEV;
    }

    err = lc709204f_write_register(config->bus, LC709204F_REG_APA, config->apa_value);
    if (err != 0) {
        return err;
    }

    int battery_type_param = lc709204f_convert_battery_type_to_param(config->battery_type);
    if (battery_type_param < 0) {
        return battery_type_param;
    }

    err = lc709204f_write_register(drv_data->i2c_spec, LC709204F_REG_CHANGE_OF_PARAMETER, battery_type_param);
    if (err != 0) {
        return err;
    }

    err = lc709204f_write_register(drv_data->i2c_spec, LC709204F_REG_TERMINATION_CURRENT_RATE, config->term_current_rate);
    if (err != 0) {
        return err;
    }

    err = lc709204f_write_register(config->bus, LC709204F_REG_EMPTY_CELL_VOLTAGE, config->empty_cell_voltage);
    if (err != 0) {
        return err;
    }


    err = lc709204f_write_register(config->bus, LC709204F_REG_IC_POWER_MODE, 0x0001);
    if (err != 0) {
        return err;
    }
}

static const struct sensor_driver_api lc709204f_api_funcs = {
    .sample_fetch = lc709204f_sample_fetch,
    .channel_get = lc709204f_channel_get,
};

#define LC709204F_INIT(inst)                                        \
    static struct lc709204f_data lc709204f_data_##inst;             \
    static const struct lc709204f_config lc709204f_config_##inst = {\
        .bus_name = I2C_DT_SPEC_INSTGET(inst),                      \
        .design_capacity = DT_INST_PROP(inst, design_capacity),     \
        .design_voltage = DT_INST_PROP(inst, design_voltage),       \
        .desired_voltage = DT_INST_PROP(inst, desired_voltage),     \
        .empty_voltage = DT_INST_PROP(inst, empty_voltage),         \
        .desired_charging_current = DT_INST_PROP(inst,              \
                desired_charging_current),                          \
        .charging_termination_current = DT_INST_PROP(inst, chg_term_current), \
        .apa_value = DT_INST_PROP(inst, apa_value),                 \
        .battery_type = DT_INST_PROP(inst, battery_type),           \
    };                                                              \
    DEVICE_DT_INST_DEFINE(inst,                                     \
            lc709204f_init,                                         \
            NULL,                                                   \
            &lc709204f_data_##inst,                                 \
            &lc709204f_config_##inst,                               \
            POST_KERNEL,                                            \
            CONFIG_SENSOR_INIT_PRIORITY,                            \
            &lc709204f_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(LC709204F_INIT)