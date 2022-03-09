#ifndef _SI1132_H_
#define _SI1132_H_

/* Si1132 HW_KEY Magic number */
#define SI1132_HW_KEY_MAGIC     0x17

/* Si1132 PART_ID */
#define SI1132_PART_ID          0x32

/* Si1132 SEQ_ID */
#define SI1132_SEQ_ID           0x08

/* Si1132 register addresses */
#define SI1132_REG_PART_ID          0x00
#define SI1132_REG_REV_ID           0x01
#define SI1132_REG_SEQ_ID           0x02
#define SI1132_REG_INT_CFG          0x03
#define SI1132_REG_IRQ_ENABLE       0x04
#define SI1132_REG_HW_KEY           0x07
#define SI1132_REG_MEAS_RATE        0x08
#define SI1132_REG_MEAS_RATE0       0x08
#define SI1132_REG_MEAS_RATE1       0x09
#define SI1132_REG_UCOEF            0x13
#define SI1132_REG_UCOEF0           0x13
#define SI1132_REG_UCOEF1           0x14
#define SI1132_REG_UCOEF2           0x15
#define SI1132_REG_UCOEF3           0x16
#define SI1132_REG_PARAM_WR         0x17
#define SI1132_REG_COMMAND          0x18
#define SI1132_REG_RESPONSE         0x20
#define SI1132_REG_IRQ_STATUS       0x21
#define SI1132_REG_ALS_VIS_DATA     0x22
#define SI1132_REG_ALS_VIS_DATA0    0x22
#define SI1132_REG_ALS_VIS_DATA1    0x23
#define SI1132_REG_ALS_IR_DATA      0x24
#define SI1132_REG_ALS_IR_DATA0     0x24
#define SI1132_REG_ALS_IR_DATA1     0x25
#define SI1132_REG_AUX_DATA         0x2C
#define SI1132_REG_AUX_DATA0        0x2C
#define SI1132_REG_AUX_DATA1        0x2D
#define SI1132_REG_PARAM_RD         0x2E
#define SI1132_REG_CHIP_STAT        0x30
#define SI1132_REG_ANA_IN_KEY       0x3B

/* Si1132 commands */
#define SI1132_CMD_PARAM_QUERY  0x80
#define SI1132_CMD_PARAM_SET    0xA0
#define SI1132_CMD_NOP          0x00
#define SI1132_CMD_RESET        0x01
#define SI1132_CMD_BUSADDR      0x02
#define SI1132_CMD_GET_CAL      0x12
#define SI1132_CMD_ALS_FORCE    0x06
#define SI1132_CMD_ALS_PAUSE    0x0A
#define SI1132_CMD_ALS_AUTO     0x0E

/* Si1132 response register error mask */
#define SI1132_RESPONSE_REG_MASK    0xF0

/* Si1132 response register error codes */
#define SI1132_RESPONSE_NO_ERROR                0x00
#define SI1132_RESPONSE_INVALID_SETTING         0x80
#define SI1132_RESPONSE_ALS_VIS_ADC_OVERFLOW    0x8C
#define SI1132_RESPONSE_ALS_IR_ADC_OVERFLOW     0x8D
#define SI1132_RESPONSE_AUX_ADC_OVERFLOW        0x8E

/* Si1132 parameter RAM offsets */
#define SI1132_PARAM_I2C_ADDR               0x00
#define SI1132_PARAM_CHLIST                 0x01
#define SI1132_PARAM_ALS_ENCODING           0x06
#define SI1132_PARAM_ALS_IR_ADCMUX          0x0E
#define SI1132_PARAM_ADCMUX                 0x0F
#define SI1132_PARAM_ALS_VIS_ADC_COUNTER    0x10
#define SI1132_PARAM_ALS_VIS_ADC_GAIN       0x11
#define SI1132_PARAM_ALS_VIS_ADC_MISC       0x12
#define SI1132_PARAM_ALS_IR_ADC_COUNTER     0x1D
#define SI1132_PARAM_ALS_IR_ADC_GAIN        0x1E
#define SI1132_PARAM_ALS_IR_ADC_MISC        0x1F

/* Si1132 register bitmasks */
/* Interrupt output enable */
#define SI1132_INT_OE               (1 << 0)

/* ALS interrupt enable */
#define SI1132_ALS_IE               (1 << 0)

/* Command interrupt status */
#define SI1132_CMD_INT              (1 << 5)

/* ALS interrupt status */
#define SI1132_ALS_INT              (1 << 0)

/* Si1132 in running mode */
#define SI1132_RUNNING              (1 << 2)

/* Si1132 in suspend mode */
#define SI1132_SUSPEND              (1 << 1)

/* Si1132 in sleep mode */
#define SI1132_SLEEP                (1 << 0)


/* Si1132 Parameter RAM bitmasks */
#define SI1132_EN_UV                (1 << 7)
#define SI1132_EN_AUX               (1 << 6)
#define SI1132_EN_ALS_IR            (1 << 5)
#define SI1132_EN_ALS_VIS           (1 << 4)

#define SI1132_ALS_IR_ALIGN         (1 << 5)
#define SI1132_ALS_VIS_ALIGN        (1 << 4)

#define SI1132_VIS_ADC_REC          (3 << 4)
#define SI1132_ALS_VIS_ADC_GAIN     (1 << 0)
#define SI1132_VIS_RANGE            (1 << 5)

#define SI1132_IR_ADC_REC           (3 << 4)
#define SI1132_ALS_IR_ADC_GAIN      (1 << 0)
#define SI1132_IR_RANGE             (1 << 5)

#endif /* _SI1132_H_ */