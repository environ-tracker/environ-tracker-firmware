#ifndef ZEPHYR_INCLUDE_DRIVERS_GPIO_RTC_RC3028_H_
#define ZEPHYR_INCLUDE_DRIVERS_GPIO_RTC_RC3028_H_

#include <zephyr/sys/timeutil.h>
#include <time.h>

struct rv3028_rtc_sec {
    uint8_t sec_one : 4;
    uint8_t sec_ten : 3;
    uint8_t nimp : 1;
} __packed;

struct rv3028_rtc_min {
    uint8_t min_one : 4;
    uint8_t min_ten : 3;
    uint8_t nimp : 1;
} __packed;

struct rv3028_rtc_hours {
    uint8_t hr_one : 4;
    uint8_t hr_ten : 2;
    uint8_t nimp : 2;
} __packed;

struct rv3028_rtc_weekday {
    uint8_t weekday : 3;
    uint8_t nimp : 5;
} __packed;

struct rv3028_rtc_date {
    uint8_t date_one : 4;
    uint8_t date_ten : 2;
    uint8_t nimp : 2;
} __packed;

struct rv3028_rtc_month {
    uint8_t month_one : 4;
    uint8_t month_ten : 1;
    uint8_t nimp : 3;
} __packed;

struct rv3028_rtc_year {
    uint8_t year_one : 4;
    uint8_t year_ten : 4;
} __packed;

struct rv3028_alm_min {
    uint8_t min_one : 4;
    uint8_t min_ten : 3;
    uint8_t n_enabled : 1;
} __packed;

struct rv3028_alm_hours {
    uint8_t hours_one : 4;
    uint8_t hours_ten : 2;
    uint8_t nimp: 1;
    uint8_t n_enabled : 1;
} __packed;

struct rv3028_alm_date {
    uint8_t date_one : 4;
    uint8_t date_ten : 2;
    uint8_t nimp : 1;
    uint8_t n_enabled : 1;
} __packed;

struct rv3028_alm_weekday {
    uint8_t weekday : 3;
    uint8_t nimp : 4;
    uint8_t n_enabled : 1;
} __packed;

union rv3028_alm_date_weekday {
    struct rv3028_alm_date date_alm;
    struct rv3028_alm_weekday weekday_alm;
};

struct rv3028_status {
    uint8_t power_on_reset : 1;
    uint8_t event : 1;
    uint8_t alarm : 1;
    uint8_t timer : 1;
    uint8_t time_update : 1;
    uint8_t backup_switch : 1;
    uint8_t clock_output_int : 1;
    uint8_t eeprom_busy : 1;
} __packed;

struct rv3028_control1 {
    uint8_t timer_freq_select : 2;
    uint8_t timer_enable : 1;
    uint8_t eeprom_refresh_disable : 1;
    uint8_t update_int_select : 1;
    uint8_t date_alm_select : 1;
    uint8_t nimp : 1;
    uint8_t timer_repeat_select : 1;
} __packed;

struct rv3028_control2 {
    uint8_t reset : 1;
    uint8_t mode_24hr : 1;
    uint8_t event_int_enable : 1;
    uint8_t alm_int_enable : 1;
    uint8_t timer_int_enable : 1;
    uint8_t time_update_int_enable : 1;
    uint8_t int_ctrl_clock_output: 1;
    uint8_t time_stamp_enable : 1;
} __packed;

struct rv3028_ctrl_registers {
    struct rv3028_control1 ctrl1;
    struct rv3028_control2 ctrl2;
} __packed;

struct rv3028_time_registers {
    struct rv3028_rtc_sec rtc_sec;
    struct rv3028_rtc_min rtc_min;
    struct rv3028_rtc_hours rtc_hours;
    struct rv3028_rtc_weekday rtc_weekday;
    struct rv3028_rtc_date rtc_date;
    struct rv3028_rtc_month rtc_month;
    struct rv3028_rtc_year rtc_year;
} __packed;

struct rv3028_alarm_registers {
    struct rv3028_alm_min alm_min;
    union rv3028_alm_date_weekday alm_date_weekday;
} __packed;

enum rv3028_register {
    REG_RTC_SEC = 0,
    REG_RTC_MIN,
    REG_RTC_HOUR,
    REG_RTC_WDAY,
    REG_RTC_DATE,
    REG_RTC_MONTH,
    REG_RTC_YEAR,
    REG_ALM_HOUR,
    REG_ALM_DATE,
    REG_TIMER_VAL0,
    REG_TIMER_VAL1,
    REG_TIMER_STATUS0,
    REG_TIMER_STATUS1,
    REG_STATUS,
    REG_CTRL1,
    REG_CTRL2,
    REG_GP_BITS,
    REG_CLOCK_INT_MASK,
    REG_EVENT_CTRL,
    REG_COUNT_TS,
    REG_SEC_TS,
    REG_MIN_TS,
    REG_HOUR_TS,
    REG_DATE_TS,
    REG_MONTH_TS,
    REG_YEAR_TS,
    REG_UNIX0,
    REG_UNIX1,
    REG_UNIX2,
    REG_UNIX3,
    REG_USER_RAM1,
    REG_USER_RAM2,
    REG_PASSWORD0,
    REG_PASSWORD1,
    REG_PASSWORD2,
    REG_PASSWORD3,
    REG_EE_ADDR,
    REG_EE_DATA,
    REG_EE_CMD,
    REG_ID,
    REG_INVAL = 0x3A,
};

/**
 * @brief Set the RTC to a given UNIX time
 * 
 * @param dev the RV3028 device pointer
 * @param unix_time UNIX time to set the RTC to
 * @return 0 on success,
 *         negative error code from I2C API,
 *         -INVAL
 */
int rv3028_rtc_set_time(const struct device *dev, time_t unix_time);

#endif /* ZEPHYR_INCLUDE_DRIVERS_GPIO_RTC_RC3028_H_ */