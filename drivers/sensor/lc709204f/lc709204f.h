#ifndef _LC709204F_H_
#define _LC709204F_H_

#include <drivers/i2c.h>

#define LC709204F_0C_VALUE      0xAAC

/* Commands */
enum lc709204f_commands {
    TIME_TO_EMPTY               = 0x03,
    BEFORE_RSOC                 = 0x04,
    TIME_TO_FULL                = 0x05,
    TSENSE1_THERM_B             = 0x06,
    INITIAL_RSOC                = 0x07,
    CELL_TEMP                   = 0x08,
    CELL_VOLTAGE                = 0x09,
    CURRENT_DIRECTION           = 0x0A,
    APA                         = 0x0B,
    APT                         = 0x0C,
    RSOC                        = 0x0D,
    TSENSE2_THERM_B             = 0x0E,
    ITE                         = 0x0F,
    IC_VERSION                  = 0x11,
    CHANGE_OF_PARAMETER         = 0x12,
    ALARM_LOW_RSOC              = 0x13,
    ALARM_LOW_VOLTAGE           = 0x14,
    IC_POWER_MODE               = 0x15,
    STATUS_BIT                  = 0x16,
    CYCLE_COUNT                 = 0x17,
    BATTERY_STATUS              = 0x19,
    NUMBER_OF_PARAMETER         = 0x1A,
    TERMINATION_CURRENT_RATE    = 0x1C,
    EMPTY_CELL_VOLTAGE          = 0x1D,
    ITE_OFFSET                  = 0x1E,
    ALARM_HIGH_VOLTAGE          = 0x1F,
    ALARM_LOW_TEMP              = 0x20,
    ALARM_HIGH_TEMP             = 0x21,
    TOTAL_RUN_TIME_LOW_2B       = 0x24,
    TOTAL_RUN_TIME_HIGH_2B      = 0x25,
    ACCUM_TEMPERATURE_LOW_2B    = 0x26,
    ACCUM_TEMPERATURE_HIGH_2B   = 0x27,
    ACCUM_RSOC_LOW_2B           = 0x28,
    ACCUM_RSOC_HIGH_2B          = 0x29,
    MAX_CELL_VOLTAGE            = 0x2A,
    MIN_CELL_VOLTAGE            = 0x2B,
    MAX_CELL_TEMP               = 0x2C,
    MIN_CELL_TEMP               = 0x2D,
    AMBIENT_TEMP                = 0x30,
    STATE_OF_HEALTH             = 0x32,
    USER_ID_LOW_2B              = 0x36,
    USER_ID_HIGH_2B             = 0x37,
};

/* LC709204F RSOC init commands */
enum lc709204f_rsoc_init_commands {
    SAMPLE_1    = 0xAA55,
    SAMPLE_2    = 0xAA56,
    SAMPLE_3    = 0xAA57,
    SAMPLE_4    = 0xAA58,
};

/* LC709204F supported battery types */
enum lc709204f_supported_battery_types {
    BATTERY_01  = 0x00,
    BATTERY_04  = 0x01,
    BATTERY_05  = 0x02,
    BATTERY_06  = 0x03,
    BATTERY_07  = 0x04,
};

/* Status bit masks */
enum lc709204f_status_bit_masks {
    TSENSE1_CTRL        = 1 << 0,
    TSENSE2_CTRL        = 1 << 1,
};

/* Status bit masks */
enum lc709204f_battery_status_bit_masks {  
    STATUS_DISCHARGING  = 1 << 6,
    STATUS_INITIALIZED  = 1 << 7,
    ALARM_LOW_TEMP      = 1 << 8,
    ALARM_LOW_RSOC      = 1 << 9,
    ALARM_LOW_VOLTAGE   = 1 << 11,
    ALARM_HIGH_TEMP     = 1 << 12,
    ALARM_HIGH_VOLTAGE  = 1 << 15,
};

struct lc709204f_data {
    /* Current cell voltage in mV */
    uint16_t voltage;
    /* Max cell voltage seen in mV */
    uint16_t max_voltage;
    /* Min cell voltage seen in mV */
    uint16_t min_voltage;
    /* Remaining capacity in units of 0.1% */
    uint16_t state_of_charge;
    /* State of health of the cell in units of 1% */
    uint16_t state_of_health;
    /* Time to full in minutes */
    uint16_t time_to_full;
    /* Time to empty in minutes */
    uint16_t time_to_empty;
    /* Cycle count (number of charge and discharge cycles) */
    uint16_t cycle_count;
    /* Current cell temperature in 0.1 degrees C offset by +0xAAC */
    uint16_t cell_temp;
    /* Max cell temperature seen in 0.1 degrees C offset by +0xAAC */
    uint16_t max_cell_temp;
    /* Min cell temperature seen in 0.1 degrees C offset by +0xAAC */
    uint16_t min_cell_temp;
    /* Current ambient temperature */
    uint16_t ambient_temp;
    /* Total run time from POR in minutes */
    uint32_t total_run_time;
};

struct lc709204f_config {
    struct i2c_dt_spec i2c;
    /* Design capacity (label capacity) of the cell in mAh */
    uint16_t design_capacity;
    /* Design capacity of the cell in mV */
    uint16_t design_voltage;
    /* Desired voltage of the cell in mV */
    uint16_t desired_voltage;
    /* Empty voltage of the cell in mV */
    uint16_t empty_voltage;
    /* Desired charging current in mA */
    uint16_t desired_charging_current;
    /* Charging termination current in uA */
    uint16_t charging_termination_current;
    /* Adjustment parameters */
    uint16_t apa_value;
    /* Battery type (a type supported by the LC709204F) */
    enum lc709204f_supported_battery_types battery_type;
};

#endif /* _LC709204F_H_ */