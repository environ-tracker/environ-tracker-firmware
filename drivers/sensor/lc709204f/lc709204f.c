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
static int lc709204f_reg_read(const struct i2c_dt_spec *spec,
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
    *value = sys_le16_to_cpu(*value);

    return 0;
}

/**
 * @brief Converts the LC709204F's temperature representation to units of 0.1 
 *        degrees celsius
 * 
 * @param lc_temperature LC709204F temperature value
 * @param sensor_val Place to put the value on success
 */
static void lc709204f_temperature_to_sensor_val(uint16_t lc_temperature, struct sensor_value *sensor_val)
{
    int temperature = lc_temperature - LC709204F_0C_VALUE;
    
    sensor_val->val1 = temperature / 10;
    sensor_val->val2 = (temperature % 10) * 100000;
}

/**
 * @brief Fetches all on device channel data at once
 * NOTE: Only SENSOR_CHAN_ALL is supported
 * 
 * @param dev Device to fetch data from
 * @param chan Channel to fetch. (Only SENSOR_CHAN_ALL supported)
 * @return 0 on success, negative error code on failure
 */
static int lc709204f_sample_fetch(const struct device *dev, 
        enum sensor_channel chan) 
{
    struct lc709204f_data *data = dev->data;
    struct lc709204f_config *config = dev->config;
    uint8_t time_buf[2] = {0};

    struct {
        enum lc709204f_commands command;
        uint16_t *dest;
    } cmds[] = {
        { CELL_VOLTAGE, &data->voltage },
        { MAX_CELL_VOLTAGE, &data->max_voltage },
        { MIN_CELL_VOLTAGE, &data->min_voltage },
        { ITE, &data->state_of_charge },
        { STATE_OF_HEALTH, &data->state_of_health },
        { TIME_TO_FULL, &data->time_to_full },
        { TIME_TO_EMPTY, &data->time_to_empty },
        { CYCLE_COUNT, &data->cycle_count },
        { CELL_TEMP, &data->cell_temp },
        { MAX_CELL_TEMP, &data->max_cell_temp },
        { MIN_CELL_TEMP, &data->min_cell_temp },
        { AMBIENT_TEMP, &data->ambient_temp },
        { TOTAL_RUN_TIME_LOW_2B, &time_buf[0] },
        { TOTAL_RUN_TIME_HIGH_2B, &time_buf[1] },
    };

    if (chan != SENSOR_CHAN_ALL) {
        LOG_ERR("Only SENSOR_CHAN_ALL is supported for fetches");
        return -ENOTSUP;
    }

    for (int i = 0; i < ARRAY_SIZE(cmds); ++i) {
        int rc = lc709204f_reg_read(&config->i2c, (uint8_t)cmds[i].command, 
                cmds[i].dest);
        if (rc != 0) {
            LOG_ERR("Failed to read channel #%d with cmd: %02x", 
                    cmds[i].command, cmds[i].dest);
            return rc;
        }
    }

    data->total_run_time = (time_buf[1] << 16) | time_buf[0];
    data->total_run_time = sys_le32_to_cpu(data->total_run_time);    

    return 0;
}

/**
 * @brief Retrieve the sensors values
 * 
 * @param dev Device to access
 * @param chan Channel to read
 * @param val Returns the sensor value read on success
 * @return 0 on success
 * @return -ENOTSUP for unsupported channels 
 */
static int lc709204f_channel_get(const struct device *dev, 
        enum sensor_channel chan, struct sensor_value *val)
{
    struct lc709204f_config *config = dev->config;
    struct lc709204f_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_GAUGE_TEMP:
        lc709204f_temperature_to_sensor_val(data->cell_temp, val);
        break;
    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        val->val1 = data->state_of_charge / 10;
        val->val2 = data->state_of_charge % 10 * 100000;
        break;
    case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
        val->val1 = config->design_capacity;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
        val->val1 = config->design_capacity * data->state_of_charge / 1000;
        val->val2 = config->design_capacity * data->state_of_charge % 1000 
                * 1000;
        break;
    case SENSOR_CHAN_GAUGE_STATE_OF_HEALTH:
        val->val1 = data->state_of_health;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_GAUGE_TIME_TO_EMPTY:
        val->val1 = data->time_to_empty;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_GAUGE_TIME_TO_FULL:
        val->val1 = data->time_to_full;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_GAUGE_CYCLE_COUNT:
        val->val1 = data->cycle_count;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE:
        val->val1 = config->design_voltage / 1000;
        val->val2 = config->design_voltage % 1000 * 1000;
        break;
    case SENSOR_CHAN_GAUGE_DESIRED_VOLTAGE:
        val->val1 = config->desired_voltage / 1000;
        val->val2 = config->desired_voltage % 1000 * 1000;
        break;
    case SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT:
        val->val1 = config->desired_charging_current;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        val->val1 = data->voltage / 1000;
        val->val2 = data->voltageq % 1000 * 1000;
        break;
    default:
        return -ENOTSUP;
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

/**
 * @brief Initialise the LC709204F after a Power On Reset (POR) event
 * 
 * @param dev Device to initialise
 * @return 0 on success, negative error code on failure
 */
static int lc709204f_init(const struct device *dev)
{
    struct lc709204f_data *data = dev->data;
    struct lc709204f_config *config = dev->config;
    int err;
    uint16_t status, tmp;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("i2c bus is not ready");
        return -ENODEV;
    }

    /* Check if POR event */
    err = lc709204f_reg_read(&config->i2c, BATTERY_STATUS, &status);
    if (err != 0) {
        return err;
    }

    if (!(status & STATUS_INITIALIZED)) {
        LOG_DBG("No POR event detected - skip device configuration");
        return 0;
    }

    /* Write APA values */
    err = lc709204f_reg_write(&config->i2c, APA, config->apa_value);
    if (err != 0) {
        return err;
    }

    /* Get battery value from battery type config */
    tmp = lc709204f_convert_battery_type_to_param(config->battery_type);
    if (tmp < 0) {
        return tmp;
    }

    /* Write battery type */
    err = lc709204f_reg_write(&config->i2c, CHANGE_OF_PARAMETER, tmp);
    if (err != 0) {
        return err;
    }

    /* Convert charging termination current in mA to 0.1C */
    tmp = (config->charging_termination_current * 100) / 
            config->design_capacity;

    /* Write termination charging rate */
    err = lc709204f_reg_write(&config->i2c, TERMINATION_CURRENT_RATE, tmp);
    if (err != 0) {
        return err;
    }

    /* Write empty cell voltage */
    err = lc709204f_reg_write(&config->i2c, EMPTY_CELL_VOLTAGE, 
            config->empty_voltage);
    if (err != 0) {
        return err;
    }

    /* Set device to Operational Mode */
    err = lc709204f_reg_write(&config->i2c, IC_POWER_MODE, 0x0001);
    if (err != 0) {
        return err;
    }

    /* Clear POR bit */
    status &= ~STATUS_INITIALIZED;
    err = lc709204f_reg_write(&config->i2c, BATTERY_STATUS, status);
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