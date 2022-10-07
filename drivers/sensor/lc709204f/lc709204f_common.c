#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include "lc709204f.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(lc709204f);

/**
 * @brief Write to a 16b register
 * 
 * @param i2c The I2C bus devicetree spec
 * @param reg_addr Register address to write to
 * @param value Value to write to register
 * @return 0 if successful, or negative error code from I2C API
 */
int lc709204f_reg_write(const struct i2c_dt_spec *i2c, 
        uint8_t reg_addr, uint16_t value) 
{
    // Addresses added to make CRC-8 calculation easier
    uint8_t buf[5] = {i2c->addr << 1, reg_addr, value, value >> 8, 0};

    // Calculate CRC-8
    buf[4] = crc8(buf, 4, 7, 0, false); 

    return i2c_write_dt(i2c, &buf[1], 4);
}


/**
 * @brief Read a 16b register value
 * NOTE: Currently the CRC-8 value is not checked
 * 
 * @param i2c The I2C bus devicetree spec
 * @param reg_addr Register address to read
 * @param value Place to put the value on success
 * @return 0 if successful, 
 *        -EBADMSG on CRC error,
 *         or negative error code from I2C API
 */
int lc709204f_reg_read(const struct i2c_dt_spec *i2c,
        uint8_t reg_addr, uint16_t *value)
{
    uint8_t buf[6] = {i2c->addr << 1, reg_addr, (i2c->addr << 1) + 1, 0, 0, 0};

    // Read 2 data bytes and crc byte
    int err = i2c_burst_read_dt(i2c, reg_addr, &buf[3], 3);
    if (err != 0) {
        return err;
    }

    uint8_t crc = crc8(buf, 5, 7, 0, false);
    if (crc != buf[5]) {
        LOG_ERR("Incorrect CRC, calc'd: 0x%02x, got: 0x%02x", crc, buf[5]);
        return -EBADMSG;
    }

    // Save register data in sys endianness
    *value = (buf[4] << 8) | buf[3];
    *value = sys_le16_to_cpu(*value);

    return 0;
}

/**
 * @brief Retrieve the Battery Status register and store it in a 
 *        status_reg struct
 * 
 * @param i2c Devices i2c devicetree spec
 * @param status Place to store register data on success
 * @return 0 on success, negative error code on failure 
 */
int lc709204f_status_reg_get(const struct i2c_dt_spec *i2c,
        struct lc709204f_battery_status_reg *status)
{
    uint16_t register_data;
    int err;

    err = lc709204f_reg_read(i2c, (uint8_t)BATTERY_STATUS, &register_data);
    if (err != 0) {
        return err;
    }

    status->high_cell_voltage = !!(register_data & ALARM_HIGH_VOLTAGE_MASK);
    status->low_cell_voltage = !!(register_data & ALARM_LOW_VOLTAGE_MASK);
    status->high_cell_temp = !!(register_data & ALARM_HIGH_TEMP_MASK);
    status->low_cell_temp = !!(register_data & ALARM_LOW_TEMP_MASK);
    status->low_rsoc = !!(register_data & ALARM_LOW_RSOC_MASK);
    status->initialized = !!(register_data & STATUS_INITIALIZED_MASK);
    status->discharging = !!(register_data & STATUS_DISCHARGING_MASK);

    return 0;
}