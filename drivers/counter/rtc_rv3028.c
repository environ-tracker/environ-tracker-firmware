#define DT_DRV_COMPAT microcrystal_rv3028

#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/rtc/rtc_rv3028.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rv3028, CONFIG_COUNTER_LOG_LEVEL);

/* Alarm channels */
#define ALARM0_ID   0

/* Size of block when writing whole struct */
#define RTC_TIME_REGISTERS_SIZE     sizeof(struct rv3028_time_registers)
#define RTC_ALARM_REGISTERS_SIZE    sizeof(struct rv3028_alarm_registers)

/* Largest block size */
#define MAX_WRITE_SIZE                  (RTC_TIME_REGISTERS_SIZE)

/* tm struct uses years since 1900 but unix time uses years since
 * 1970. RV3028 default year is '0' so the offset is 70
 */
#define UNIX_YEAR_OFFSET		70

/* Macro used to decode BCD to UNIX time to avoid potential copy and paste
 * errors.
 */
#define RTC_BCD_DECODE(reg_prefix) (reg_prefix##_one + reg_prefix##_ten * 10)

struct rv3028_config {
	struct counter_config_info generic;
    struct i2c_dt_spec bus;
};

struct rv3028_data {
	const struct device *rv3028;
    bool time_initialised;
	struct k_sem lock;
    struct rv3028_status status;
    struct rv3028_ctrl_registers control;
	struct rv3028_time_registers registers;
};

/** @brief Convert BCD time in device registers to UNIX time
 *
 * @param dev the RV3028 device pointer.
 *
 * @retval returns unix time.
 */
static time_t decode_rtc(const struct device *dev)
{
	struct rv3028_data *data = dev->data;
	time_t time_unix = 0;
	struct tm time = { 0 };

	time.tm_sec = RTC_BCD_DECODE(data->registers.rtc_sec.sec);
	time.tm_min = RTC_BCD_DECODE(data->registers.rtc_min.min);
	time.tm_hour = RTC_BCD_DECODE(data->registers.rtc_hours.hr);
	time.tm_mday = RTC_BCD_DECODE(data->registers.rtc_date.date);
	time.tm_wday = data->registers.rtc_weekday.weekday;
	/* tm struct starts months at 0, rv3028 starts at 1 */
	time.tm_mon = RTC_BCD_DECODE(data->registers.rtc_month.month) - 1;
	/* tm struct uses years since 1900 but unix time uses years since 1970 */
	time.tm_year = RTC_BCD_DECODE(data->registers.rtc_year.year) +
		UNIX_YEAR_OFFSET;

	time_unix = timeutil_timegm(&time);

	LOG_DBG("Unix time is %d\n", (uint32_t)time_unix);

	return time_unix;
}

/** @brief Encode time struct tm into rv3028 rtc registers
 *
 * @param dev the RV3028 device pointer.
 * @param time_buffer tm struct containing time to be encoded into rv3028
 * registers.
 *
 * @retval return 0 on success, or a negative error code from invalid
 * parameter.
 */
static int encode_rtc(const struct device *dev, struct tm *time_buffer)
{
	struct rv3028_data *data = dev->data;
	uint8_t month;
	uint8_t year_since_epoch;

	/* In a tm struct, months start at 0, rv3028 starts with 1 */
	month = time_buffer->tm_mon + 1;

	if (time_buffer->tm_year < UNIX_YEAR_OFFSET) {
		return -EINVAL;
	}
	year_since_epoch = time_buffer->tm_year - UNIX_YEAR_OFFSET;

	data->registers.rtc_sec.sec_one = time_buffer->tm_sec % 10;
	data->registers.rtc_sec.sec_ten = time_buffer->tm_sec / 10;
	data->registers.rtc_min.min_one = time_buffer->tm_min % 10;
	data->registers.rtc_min.min_ten = time_buffer->tm_min / 10;
	data->registers.rtc_hours.hr_one = time_buffer->tm_hour % 10;
	data->registers.rtc_hours.hr_ten = time_buffer->tm_hour / 10;
	data->registers.rtc_weekday.weekday = time_buffer->tm_wday;
	data->registers.rtc_date.date_one = time_buffer->tm_mday % 10;
	data->registers.rtc_date.date_ten = time_buffer->tm_mday / 10;
	data->registers.rtc_month.month_one = month % 10;
	data->registers.rtc_month.month_ten = month / 10;
	data->registers.rtc_year.year_one = year_since_epoch % 10;
	data->registers.rtc_year.year_ten = year_since_epoch / 10;

	return 0;
}

/** @brief Reads single register from RV3028
 *
 * @param dev the RV3028 device pointer.
 * @param addr register address.
 * @param val pointer to uint8_t that will contain register value if
 * successful.
 *
 * @retval return 0 on success, or a negative error code from an I2C
 * transaction.
 */
static int read_register(const struct device *dev, uint8_t addr, uint8_t *val)
{
	const struct rv3028_config *cfg = dev->config;

	return i2c_write_read_dt(&cfg->bus, &addr, sizeof(addr), val, 1);
}

/** @brief Read registers from device and populate rv3028_registers struct
 *
 * @param dev the RV3028 device pointer.
 * @param unix_time pointer to time_t value that will contain unix time if
 * successful.
 *
 * @retval return 0 on success, or a negative error code from an I2C
 * transaction.
 */
static int read_time(const struct device *dev, time_t *unix_time)
{
	struct rv3028_data *data = dev->data;
	const struct rv3028_config *cfg = dev->config;
	uint8_t addr = REG_RTC_SEC;

	int rc = i2c_write_read_dt(&cfg->bus, &addr, sizeof(addr), 
            &data->registers, RTC_TIME_REGISTERS_SIZE);

	if (rc >= 0) {
		*unix_time = decode_rtc(dev);
	}

	return rc;
}

/** @brief Write a single register to RV3028
 *
 * @param dev the RV3028 device pointer.
 * @param addr register address.
 * @param value Value that will be written to the register.
 *
 * @retval return 0 on success, or a negative error code from an I2C
 * transaction or invalid parameter.
 */
static int write_register(const struct device *dev, enum rv3028_register addr, 
        uint8_t value)
{
	const struct rv3028_config *cfg = dev->config;

	uint8_t time_data[2] = {addr, value};

	return i2c_write_dt(&cfg->bus, time_data, sizeof(time_data));
}

/** @brief Write a full time struct to RV3028 registers.
 *
 * @param dev the RV3028 device pointer.
 * @param addr first register address to write to, should be REG_RTC_SEC,
 * REG_ALM0_SEC or REG_ALM0_SEC.
 * @param size size of data struct that will be written.
 *
 * @retval return 0 on success, or a negative error code from an I2C
 * transaction or invalid parameter.
 */
static int write_data_block(const struct device *dev, 
        enum rv3028_register addr, uint8_t size)
{
	struct rv3028_data *data = dev->data;
	const struct rv3028_config *cfg = dev->config;
	uint8_t time_data[MAX_WRITE_SIZE + 1];
	uint8_t *write_block_start;

	if (size > MAX_WRITE_SIZE) {
		return -EINVAL;
	}

	if (addr >= REG_INVAL) {
		return -EINVAL;
	}

	if (addr == REG_RTC_SEC) {
		write_block_start = (uint8_t *)&data->registers;
    } else {
		return -EINVAL;
	}

	/* Load register address into first byte then fill in data values */
	time_data[0] = addr;
	memcpy(&time_data[1], write_block_start, size);

	return i2c_write_dt(&cfg->bus, time_data, size + 1);
}

/** @brief Sets the correct weekday.
 *
 * If the time is never set then the device defaults to 1st January 1970
 * but with the wrong weekday set. This function ensures the weekday is
 * correct in this case.
 *
 * @param dev the RV3028 device pointer.
 * @param unix_time pointer to unix time that will be used to work out the weekday
 *
 * @retval return 0 on success, or a negative error code from an I2C
 * transaction or invalid parameter.
 */
static int set_day_of_week(const struct device *dev, time_t *unix_time)
{
	struct rv3028_data *data = dev->data;
	struct tm time_buffer = { 0 };
	int rc = 0;

	gmtime_r(unix_time, &time_buffer);

	if (time_buffer.tm_wday != 0) {
		data->registers.rtc_weekday.weekday = time_buffer.tm_wday;
		rc = write_register(dev, REG_RTC_WDAY,
			    *((uint8_t *)(&data->registers.rtc_weekday)));
	} else {
		rc = -EINVAL;
	}

	return rc;
}

int rv3028_rtc_set_time(const struct device *dev, time_t unix_time)
{
	struct rv3028_data *data = dev->data;
	struct tm time_buffer = { 0 };
	int rc = 0;

	if (unix_time > UINT32_MAX) {
		LOG_ERR("Unix time must be 32-bit");
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER);

	/* Convert unix_time to civil time */
	gmtime_r(&unix_time, &time_buffer);
	LOG_DBG("Desired time is %d-%d-%d %d:%d:%d\n", (time_buffer.tm_year + 1900),
		(time_buffer.tm_mon + 1), time_buffer.tm_mday, time_buffer.tm_hour,
		time_buffer.tm_min, time_buffer.tm_sec);

	/* Encode time */
	rc = encode_rtc(dev, &time_buffer);
	if (rc < 0) {
		goto out;
	}

	/* Write to device */
	rc = write_data_block(dev, REG_RTC_SEC, RTC_TIME_REGISTERS_SIZE);

out:
	k_sem_give(&data->lock);

	return rc;
}

static int rv3028_counter_start(const struct device *dev)
{
    // The RV3028 auto-starts at power up and cannot be stopped
	return 0;
}

static int rv3028_counter_stop(const struct device *dev)
{
    // The RV3028 auto-starts at power up and cannot be stopped
	return -ENOTSUP;
}

static int rv3028_counter_get_value(const struct device *dev, uint32_t *ticks)
{
	struct rv3028_data *data = dev->data;
	time_t unix_time;
	int rc;

	k_sem_take(&data->lock, K_FOREVER);

	/* Get time */
	rc = read_time(dev, &unix_time);

	/* Convert time to ticks */
	if (rc >= 0) {
		*ticks = unix_time;
	}

	k_sem_give(&data->lock);

	return rc;
}

static int rv3028_counter_set_alarm(const struct device *dev, uint8_t alarm_id,
        const struct counter_alarm_cfg *alarm_cfg)
{
    return -ENOTSUP;
}

static int rv3028_counter_cancel_alarm(const struct device *dev, 
        uint8_t alarm_id)
{
    return -ENOTSUP;
}

static uint32_t rv3028_counter_get_pending_int(const struct device *dev)
{
    return -ENOTSUP;
}

static int rv3028_counter_set_top_value(const struct device *dev,
        const struct counter_top_cfg *cfg)
{
	return -ENOTSUP;
}

static uint32_t rv3028_counter_get_top_value(const struct device *dev)
{
	return UINT32_MAX;
}

static int rv3028_init(const struct device *dev)
{
	struct rv3028_data *data = dev->data;
	const struct rv3028_config *cfg = dev->config;
	int rc = 0;
	time_t unix_time = 0;

	/* Initialize and take the lock */
	k_sem_init(&data->lock, 0, 1);

	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C device %s is not ready", cfg->bus.bus->name);
		rc = -ENODEV;
		goto out;
	}

    rc = read_register(dev, REG_STATUS, (uint8_t *)&data->status);
    if (rc < 0) {
        goto out;
    }

    rc = read_time(dev, &unix_time);
    if (rc < 0) {
        goto out;
    }

    if (data->status.power_on_reset) {
        data->status.power_on_reset = 0;

        rc = write_register(dev, REG_STATUS, *(uint8_t *)&data->status);
        if (rc < 0) {
            goto out;
        }

        rc = set_day_of_week(dev, &unix_time);
    }

out:
	k_sem_give(&data->lock);

    LOG_DBG("Finished initialising RV3028: %d, unix time: %lld", rc, unix_time);

	return rc;
}

static const struct counter_driver_api rv3028_api = {
	.start = rv3028_counter_start,
	.stop = rv3028_counter_stop,
	.get_value = rv3028_counter_get_value,
	.set_alarm = rv3028_counter_set_alarm,
	.cancel_alarm = rv3028_counter_cancel_alarm,
	.set_top_value = rv3028_counter_set_top_value,
	.get_pending_int = rv3028_counter_get_pending_int,
	.get_top_value = rv3028_counter_get_top_value,
};


#define RV3028_INIT(inst)                                       \
    static struct rv3028_data rv3028_data_##inst;               \
    static const struct rv3028_config rv3028_config_##inst = {  \
        .generic = {                                            \
            .max_top_value = UINT32_MAX,                        \
            .freq = 1,                                          \
            .flags = COUNTER_CONFIG_INFO_COUNT_UP,              \
            .channels = 1,                                      \
        },                                                      \
        .bus = I2C_DT_SPEC_INST_GET(inst),                      \
    };                                                          \
    DEVICE_DT_INST_DEFINE(inst,                                 \
            rv3028_init,                                        \
            NULL,                                               \
            &rv3028_data_##inst,                                \
            &rv3028_config_##inst,                              \
            POST_KERNEL,                                        \
            CONFIG_SENSOR_INIT_PRIORITY,                       \
            &rv3028_api)

DT_INST_FOREACH_STATUS_OKAY(RV3028_INIT);