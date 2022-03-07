#define DT_DRV_COMPAT semtech_sx1502

#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/gpio.h>
#include <drivers/gpio/gpio_sx1502.h>
#include <drivers/i2c.h>
#include <sys/byteorder.h>
#include <sys/util.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(sx1502, CONFIG_GPIO_LOG_LEVEL);

#include "gpio_utils.h"

#define NUM_PINS 8

#define ALL_PINS ((uint8_t)BIT_MASK(NUM_PINS))

#define RESET_DELAY_MS 7

struct sx1502_pin_state {
    uint8_t data; /* 0x00 */
    uint8_t dir; /* 0x01 */
    uint8_t pull_up; /* 0x02 */
    uint8_t pull_down; /* 0x03 */
} __packed;

struct sx1502_irq_state {
    uint8_t interrupt_mask; /* 0x05 */
    uint16_t interrupt_sense; /* 0x06, 0x07 */
} __packed;

/** Runtime driver data */
struct sx1502_drv_data {
    /* gpio_driver_data needs to be first */
    struct gpio_driver_data common;
    const struct device *i2c_master;
    struct sx1502_pin_state pin_state;
    struct k_sem lock;

#ifdef CONFIG_GPIO_SX1502_INTERRUPT
    const struct device *gpio_int;
    struct gpio_callback gpio_cb;
    struct k_work work;
    struct sx1502_irq_state irq_state;
    const struct device *dev;
    /* user ISR cb */
    sys_slist_t cb;
#endif /* CONFIG_GPIO_SX1502_INTERRUPT */
};

/** Configuration data */
struct sx1502_config {
    /* gpio_driver_config needs to be first */
    struct gpio_driver_config common;
    const char *i2c_master_dev_name;
#ifdef CONFIG_GPIO_SX1502_INTERRUPT
    const char *gpio_int_dev_name;
    gpio_pin_t gpio_pin;
    gpio_dt_flags_t gpio_flags;
#endif /* CONFIG_GPIO_SX1502_INTERRUPT */
    uint16_t i2c_slave_addr;
};

/* Pin configuration register addresses */
enum {
    SX1502_REG_DATA             = 0x00,
    SX1502_REG_DIR              = 0x01,
    SX1502_REG_PULL_UP          = 0x02,
    SX1502_REG_PULL_DOWN        = 0x03,
    SX1502_REG_INTERRUPT_MASK   = 0x05,
    SX1502_REG_INTERRUPT_SENSE  = 0x06,
    SX1502_REG_INTERRUPT_SOURCE = 0x08,
};

/* Edge sensitivity types */
enum {
    SX1502_EDGE_NONE = 0x00,
    SX1502_EDGE_RISING = 0x01,
    SX1502_EDGE_FALLING = 0x02,
    SX1502_EDGE_BOTH = 0x03,
};

/**
 * @brief Write a big-endian word to an internal address of an I2C slave.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param dev_addr Address of the I2C device for writing.
 * @param reg_addr Address of the internal register being written.
 * @param value Value to be written to internal register.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i2c_reg_write_word_be(const struct device *dev,
					uint16_t dev_addr,
					uint8_t reg_addr, uint16_t value)
{
	uint8_t tx_buf[3] = { reg_addr, value >> 8, value & 0xff };

	return i2c_write(dev, tx_buf, 3, dev_addr);
}

/**
 * @brief Write a big-endian byte to an internal address of an I2C slave.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param dev_addr Address of the I2C device for writing.
 * @param reg_addr Address of the internal register being written.
 * @param value Value to be written to internal register.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i2c_reg_write_byte_be(const struct device *dev,
					uint16_t dev_addr,
					uint8_t reg_addr, uint8_t value)
{
	uint8_t tx_buf[3] = { reg_addr, value };

	return i2c_write(dev, tx_buf, 2, dev_addr);
}

#ifdef CONFIG_GPIO_SX1502_INTERRUPT
static int sx1502_handle_interrupt(const struct device *dev)
{
	const struct sx1502_config *cfg = dev->config;
	struct sx1502_drv_data *drv_data = dev->data;
	int ret = 0;
	uint8_t int_source;
	uint8_t cmd = SX1502_REG_INTERRUPT_SOURCE;

	k_sem_take(&drv_data->lock, K_FOREVER);

	ret = i2c_write_read(drv_data->i2c_master, cfg->i2c_slave_addr,
			     &cmd, sizeof(cmd),
			     (uint8_t *)&int_source, sizeof(int_source));
	if (ret != 0) {
		goto out;
	}

	/* reset interrupts before invoking callbacks */
	ret = i2c_reg_write_byte_be(drv_data->i2c_master, cfg->i2c_slave_addr,
				    SX1509B_REG_INTERRUPT_SOURCE, int_source);

out:
	k_sem_give(&drv_data->lock);

	if (ret == 0) {
		gpio_fire_callbacks(&drv_data->cb, dev, int_source);
	}

	return ret;
}

static void sx1509b_work_handler(struct k_work *work)
{
	struct sx1502_drv_data *drv_data =
		CONTAINER_OF(work, struct sx1502_drv_data, work);

	sx1502_handle_interrupt(drv_data->dev);
}

static void sx1509_int_cb(const struct device *dev,
			   struct gpio_callback *gpio_cb,
			   uint32_t pins)
{
	struct sx1502_drv_data *drv_data = CONTAINER_OF(gpio_cb,
		struct sx1502_drv_data, gpio_cb);

	ARG_UNUSED(pins);

	k_work_submit(&drv_data->work);
}
#endif /* CONFIG_GPIO_SX1502_INTERRUPT */

static int write_pin_state(const struct sx1502_config *cfg,
			   struct sx1502_drv_data *drv_data,
			   struct sx1502_pin_state *pins)
{
	int rc;

    rc = i2c_write(drv_data->i2c_master, &pins->data, sizeof(*pins),
                cfg->i2c_slave_addr);

	return rc;
}

static int sx1502_config(const struct device *dev,
			  gpio_pin_t pin,
			  gpio_flags_t flags)
{
	const struct sx1502_config *cfg = dev->config;
	struct sx1502_drv_data *drv_data = dev->data;
	struct sx1502_pin_state *pins = &drv_data->pin_state;
	int rc = 0;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	/* Zephyr currently defines drive strength support based on
	 * the behavior and capabilities of the Nordic GPIO
	 * peripheral: strength defaults to low but can be set high,
	 * and is controlled independently for output levels.
	 *
	 * SX150x defaults to high strength, and does not support
	 * different strengths for different levels.
	 *
	 * Until something more general is available reject any
	 * attempt to set a non-default drive strength.
	 */
	if ((flags & GPIO_DS_ALT) != 0) {
		return -ENOTSUP;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	if ((flags & GPIO_SINGLE_ENDED) != 0) {
        /* Single ended not supported */
        rc = -ENOTSUP;
        goto out;
	}

	if ((flags & GPIO_PULL_UP) != 0) {
		pins->pull_up |= BIT(pin);
	} else {
		pins->pull_up &= ~BIT(pin);
	}
	if ((flags & GPIO_PULL_DOWN) != 0) {
		pins->pull_down |= BIT(pin);
	} else {
		pins->pull_down &= ~BIT(pin);
	}

	if ((flags & GPIO_OUTPUT) != 0) {
		pins->dir &= ~BIT(pin);
		if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			pins->data &= ~BIT(pin);
		} else if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			pins->data |= BIT(pin);
		}
	} else {
		pins->dir |= BIT(pin);
	}

	if ((flags & GPIO_INT_DEBOUNCE) != 0) {
		/* debouncing not supported */
        rc = -ENOTSUP;
        goto out;
	}

	LOG_DBG("CFG %u %x : PU %02x ; PD %02x ; DIR %02x ; DAT %02x",
		pin, flags,
		pins->pull_up, pins->pull_down,
		pins->dir, pins->data);

	rc = write_pin_state(cfg, drv_data, pins, data_first);

out:
	k_sem_give(&drv_data->lock);
	return rc;
}

static int port_get(const struct device *dev,
		    gpio_port_value_t *value)
{
	const struct sx1502_config *cfg = dev->config;
	struct sx1502_drv_data *drv_data = dev->data;
	uint8_t pin_data;
	int rc = 0;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	uint8_t cmd = SX1502_REG_DATA;

	rc = i2c_write_read(drv_data->i2c_master, cfg->i2c_slave_addr,
			    &cmd, sizeof(cmd),
			    &pin_data, sizeof(pin_data));
	LOG_DBG("read %02x got %d", pin_data, rc);
	if (rc != 0) {
		goto out;
	}

	*value = pin_data;

out:
	k_sem_give(&drv_data->lock);
	return rc;
}

static int port_write(const struct device *dev,
		      gpio_port_pins_t mask,
		      gpio_port_value_t value,
		      gpio_port_value_t toggle)
{
	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	const struct sx1502_config *cfg = dev->config;
	struct sx1502_drv_data *drv_data = dev->data;
	void *data = &drv_data->pin_state.data;
	uint8_t *outp = data;

	__ASSERT_NO_MSG(IS_PTR_ALIGNED(data, uint8_t));

	k_sem_take(&drv_data->lock, K_FOREVER);

	uint8_t orig_out = *outp;
	uint8_t out = ((orig_out & ~mask) | (value & mask)) ^ toggle;
	int rc = i2c_reg_write_byte_be(drv_data->i2c_master, cfg->i2c_slave_addr,
				       SX1509B_REG_DATA, out);
	if (rc == 0) {
		*outp = out;
	}

	k_sem_give(&drv_data->lock);

	LOG_DBG("write %02x msk %02x val %02x => %02x: %d", orig_out, mask, value, out, rc);

	return rc;
}

static int port_set_masked(const struct device *dev,
			   gpio_port_pins_t mask,
			   gpio_port_value_t value)
{
	return port_write(dev, mask, value, 0);
}

static int port_set_bits(const struct device *dev,
			 gpio_port_pins_t pins)
{
	return port_write(dev, pins, pins, 0);
}

static int port_clear_bits(const struct device *dev,
			   gpio_port_pins_t pins)
{
	return port_write(dev, pins, 0, 0);
}

static int port_toggle_bits(const struct device *dev,
			    gpio_port_pins_t pins)
{
	return port_write(dev, 0, 0, pins);
}

static int pin_interrupt_configure(const struct device *dev,
				   gpio_pin_t pin,
				   enum gpio_int_mode mode,
				   enum gpio_int_trig trig)
{
	int rc = 0;

	if (!IS_ENABLED(CONFIG_GPIO_SX1502_INTERRUPT)
	    && (mode != GPIO_INT_MODE_DISABLED)) {
		return -ENOTSUP;
	}

#ifdef CONFIG_GPIO_SX1502_INTERRUPT
	/* Device does not support level-triggered interrupts. */
	if (mode == GPIO_INT_MODE_LEVEL) {
		return -ENOTSUP;
	}

	const struct sx1502_config *cfg = dev->config;
	struct sx1502_drv_data *drv_data = dev->data;
	struct sx1502_irq_state *irq = &drv_data->irq_state;
	struct {
		uint8_t reg;
		struct sx1502_irq_state irq;
	} __packed irq_buf;

	/* Only level triggered interrupts are supported, and those
	 * only if interrupt support is enabled.
	 */
	if (IS_ENABLED(CONFIG_GPIO_SX1502_INTERRUPT)) {
		if (mode == GPIO_INT_MODE_LEVEL) {
			return -ENOTSUP;
		}
	} else if (mode != GPIO_INT_MODE_DISABLED) {
		return -ENOTSUP;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	irq->interrupt_sense &= ~(SX1502_EDGE_BOTH << (pin * 2));
	if (mode == GPIO_INT_MODE_DISABLED) {
		irq->interrupt_mask |= BIT(pin);
	} else { /* GPIO_INT_MODE_EDGE */
		irq->interrupt_mask &= ~BIT(pin);
		if (trig == GPIO_INT_TRIG_BOTH) {
			irq->interrupt_sense |= (SX1502_EDGE_BOTH <<
								(pin * 2));
		} else if (trig == GPIO_INT_TRIG_LOW) {
			irq->interrupt_sense |= (SX1502_EDGE_FALLING <<
								(pin * 2));
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			irq->interrupt_sense |= (SX1502_EDGE_RISING <<
								(pin * 2));
		}
	}

	irq_buf.reg = SX1509B_REG_INTERRUPT_MASK;
	irq_buf.irq.interrupt_mask = irq->interrupt_mask;
	irq_buf.irq.interrupt_sense = sys_cpu_to_be16(irq->interrupt_sense);

	rc = i2c_write(drv_data->i2c_master, &irq_buf.reg, sizeof(irq_buf),
		       cfg->i2c_slave_addr);

	k_sem_give(&drv_data->lock);
#endif /* CONFIG_GPIO_SX1509B_INTERRUPT */

	return rc;
}


/**
 * @brief Initialization function of SX1509B
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
static int sx1502_init(const struct device *dev)
{
	const struct sx1502_config *cfg = dev->config;
	struct sx1502_drv_data *drv_data = dev->data;
	int rc;

	drv_data->i2c_master = device_get_binding(cfg->i2c_master_dev_name);
	if (!drv_data->i2c_master) {
		LOG_ERR("%s: no bus %s", dev->name,
			cfg->i2c_master_dev_name);
		rc = -EINVAL;
		goto out;
	}

#ifdef CONFIG_GPIO_SX1502_INTERRUPT
	drv_data->dev = dev;

	drv_data->gpio_int = device_get_binding(cfg->gpio_int_dev_name);
	if (!drv_data->gpio_int) {
		rc = -ENOTSUP;
		goto out;
	}
	k_work_init(&drv_data->work, sx1502_work_handler);

	gpio_pin_configure(drv_data->gpio_int, cfg->gpio_pin,
			   GPIO_INPUT | cfg->gpio_flags);
	gpio_pin_interrupt_configure(drv_data->gpio_int, cfg->gpio_pin,
				     GPIO_INT_EDGE_TO_ACTIVE);

	gpio_init_callback(&drv_data->gpio_cb, sx1502_int_cb,
			   BIT(cfg->gpio_pin));
	gpio_add_callback(drv_data->gpio_int, &drv_data->gpio_cb);

	drv_data->irq_state = (struct sx1502_irq_state) {
		.interrupt_mask = ALL_PINS,
	};
#endif

	k_sleep(K_MSEC(RESET_DELAY_MS));

	/* Reset state mediated by initial configuration */
	drv_data->pin_state = (struct sx1502_pin_state) {
		.dir = (ALL_PINS
			& ~(DT_INST_PROP(0, init_out_low)
			    | DT_INST_PROP(0, init_out_high))),
		.data = (ALL_PINS
			 & ~DT_INST_PROP(0, init_out_low)),
	};

	rc = i2c_reg_write_word_be(drv_data->i2c_master,
					cfg->i2c_slave_addr,
					SX1509B_REG_DATA,
					drv_data->pin_state.data);

	if (rc == 0) {
		rc = i2c_reg_write_word_be(drv_data->i2c_master,
					   cfg->i2c_slave_addr,
					   SX1509B_REG_DIR,
					   drv_data->pin_state.dir);
	}

out:
	if (rc != 0) {
		LOG_ERR("%s init failed: %d", dev->name, rc);
	} else {
		LOG_INF("%s init ok", dev->name);
	}
	k_sem_give(&drv_data->lock);
	return rc;
}


#ifdef CONFIG_GPIO_SX1509B_INTERRUPT
static int gpio_sx1502_manage_callback(const struct device *dev,
					  struct gpio_callback *callback,
					  bool set)
{
	struct sx1502_drv_data *data = dev->data;

	return gpio_manage_callback(&data->cb, callback, set);
}
#endif

static const struct gpio_driver_api api_table = {
	.pin_configure = sx1502_config,
	.port_get_raw = port_get,
	.port_set_masked_raw = port_set_masked,
	.port_set_bits_raw = port_set_bits,
	.port_clear_bits_raw = port_clear_bits,
	.port_toggle_bits = port_toggle_bits,
	.pin_interrupt_configure = pin_interrupt_configure,
#ifdef CONFIG_GPIO_SX1502_INTERRUPT
	.manage_callback = gpio_sx1502_manage_callback,
#endif
};

static const struct sx1502_config sx1502_cfg = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(0),
	},
	.i2c_master_dev_name = DT_INST_BUS_LABEL(0),
#ifdef CONFIG_GPIO_SX1502_INTERRUPT
	.gpio_int_dev_name = DT_INST_GPIO_LABEL(0, nint_gpios),
	.gpio_pin = DT_INST_GPIO_PIN(0, nint_gpios),
	.gpio_flags = DT_INST_GPIO_FLAGS(0, nint_gpios),
#endif
	.i2c_slave_addr = DT_INST_REG_ADDR(0),
};

static struct sx1502_drv_data sx1502_drvdata = {
	.lock = Z_SEM_INITIALIZER(sx1502_drvdata.lock, 1, 1),
};

DEVICE_DT_INST_DEFINE(0, sx1502_init, NULL,
		 &sx1502_drvdata, &sx1502_cfg,
		 POST_KERNEL, CONFIG_GPIO_SX1502_INIT_PRIORITY,
		 &api_table);
