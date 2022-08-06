#include <kernel.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>

#include <logging/log.h>
LOG_MODULE_DECLARE(lc709204f, CONFIG_SENSOR_LOG_LEVEL);

#include "lc709204f.h"

#define DT_DRV_COMPAT onsemi_lc709204f

/**
 * @brief Sets the given trigger
 * 
 * @param dev Device to set trigger for
 * @param trig Trigger type to set
 * @param handler Handler to call on trigger event
 * @return 0 on success, negative error code on failure
 */
int lc709204f_trigger_set(const struct device *dev, 
        const struct sensor_trigger *trig, sensor_trigger_handler_t handler)
{
    struct lc709204f_data *data = dev->data;
    const struct lc709204f_config *config = dev->config;
    int err = -ENOTSUP;
    
    if (!config->trig_enabled) {
        LOG_ERR("trigger_set operation not supported");
        return -ENOTSUP;
    }

    LOG_INF("Setting trigger type %d", trig->chan);


    k_mutex_lock(&data->threshold_mutex, K_MSEC(1));

    if (trig->chan == SENSOR_CHAN_GAUGE_VOLTAGE) {
        data->handler_alarm_voltage = handler;

        err = lc709204f_reg_write(&config->i2c, ALARM_HIGH_VOLTAGE, 
                data->alarm_high_voltage_threshold);
        if (err != 0) {
            goto exit;
        }
        err = lc709204f_reg_write(&config->i2c, ALARM_LOW_VOLTAGE, 
                data->alarm_low_voltage_threshold);

    } else if (trig->chan == SENSOR_CHAN_GAUGE_TEMP) {
        data->handler_alarm_temp = handler;

        err = lc709204f_reg_write(&config->i2c, ALARM_HIGH_TEMP, 
                data->alarm_high_temp_threshold);
        if (err != 0) {
            goto exit;
        }

        err = lc709204f_reg_write(&config->i2c, ALARM_LOW_TEMP, 
                data->alarm_low_temp_threshold);

    } else if (trig->chan == SENSOR_CHAN_GAUGE_STATE_OF_CHARGE) {
        data->handler_alarm_rsoc = handler;

        err = lc709204f_reg_write(&config->i2c, ALARM_LOW_RSOC, 
                data->alarm_low_rsoc_threshold);
    }

exit:

    k_mutex_unlock(&data->threshold_mutex);

    return -ENOTSUP;
}


/**
 * @brief Clears the given interrupt flags from the Battery Status register
 * 
 * @param dev Device to clear the interrupts for
 * @param flags_to_clear Interrupt flags to clear
 * @return 0 on success, negative error code on failure
 */
int lc709204f_clear_interrupt_flag(const struct device *dev, 
        uint16_t flags_to_clear)
{
    uint16_t status;
    int err;

    err = lc709204f_reg_read(&dev->config->i2c, BATTERY_STATUS, &status);
    if (err != 0) {
        return err;
    }

    return lc709204f_reg_write(&dev->config->i2c, BATTERY_STATUS, 
            status & ~flags_to_clear);
}


/**
 * @brief Handles a gpio interrupt for the device
 * NOTE: This handler DOES NOT clear the interrupt flag. This is left to the 
 *       application code to do via the `lc709204f_clear_interrupt_flag()` 
 *       function. This has been done so that advantage can be taken of the 
 *       LC709204F's none latched interrupt pin.
 * 
 * @param dev Device to handle interrupt for
 */
static void lc709204f_handle_interrupt(const struct device *dev)
{
    struct lc709204f_data *data = dev->data;
    struct sensor_trigger int_trigger = {
        .type = SENSOR_TRIG_THRESHOLD,
    };
    const struct lc709204f_config *config = dev->config;
    struct lc709204f_battery_status_reg status;

    while (1) {
        if (lc709204f_status_reg_get(&config->i2c, &status) != 0) {
            return;
        }

        /* Check if there's interrupts to handle */
        if ((status.high_cell_voltage == 0) && (status.low_cell_voltage == 0) &&
                (status.high_cell_temp == 0) && (status.low_cell_temp == 0) &&
                (status.low_rsoc == 0)) {
            break;
        }

        /* Handle interrupts as required */
        if ((status.high_cell_voltage || status.low_cell_voltage) && 
                (data->handler_alarm_voltage != NULL)) {
            data->handler_alarm_voltage(dev, &int_trigger);
        }

        if ((status.high_cell_temp || status.low_cell_temp) && 
                (data->handler_alarm_temp != NULL)) {
            data->handler_alarm_temp(dev, &int_trigger);
        }

        if (status.low_rsoc && (data->handler_alarm_rsoc != NULL)) {
            data->handler_alarm_rsoc(dev, &int_trigger);
        }
    }
}


/**
 * @brief gpio callback for device. Signals to the handling thread to handle 
 *        interrupt
 * 
 * @param dev Device callback has been called for
 * @param cb Callback data
 */
static void lc709204f_gpio_callback(const struct device *dev, 
        struct gpio_callback *cb, uint32_t pins) 
{
    // NOTE: can this be *data = dev->data??
    struct lc709204f_data *data = 
            CONTAINER_OF(cb, struct lc709204f_data, gpio_cb);
    const struct lc709204f_config *config = data->dev->config;

    ARG_UNUSED(pins);

    gpio_pin_interrupt_configure_dt(&config->gpio_int, GPIO_INT_DISABLE);

#if defined(CONFIG_LC709204F_TRIGGER_OWN_THREAD) 
    k_sem_give(&data->gpio_sem);
#elif defined(CONFIG_LC709204F_TRIGGER_GLOBAL_THREAD)
    k_work_submit(&data->work);
#endif /* CONFIG_LC709204F_TRIGGER_OWN_THREAD */
}


#ifdef CONFIG_LC709204F_TRIGGER_OWN_THREAD
/**
 * @brief Thread to handle device interrupts
 * 
 * @param data Device data
 */
static void lc709204f_thread(struct lc709204f_data *data)
{
    while (1) {
        k_sem_take(&data->gpio_sem, K_FOREVER);
        lc709204f_handle_interrupt(data->dev);
    }
}
#endif /* CONFIG_LC709204F_TRIGGER_OWN_THREAD */


#ifdef CONFIG_LC709204F_TRIGGER_GLOBAL_THREAD
/**
 * @brief Work to be submitted to submitted
 */
static void lc709204f_work_cb(struct k_work *work)
{
    struct lc709204f_data *data = 
            CONTAINER_OF(work, struct lc709204f_data, work);

    lc709204f_handle_interrupt(data->dev);
}
#endif /* CONFIG_LC709204F_TRIGGER_GLOBAL_THREAD */


/**
 * @brief Initialises the devices interrupt related configurations
 * 
 * @param dev Device to initialise interrupts for
 * @return 0 on success, negative error code on failure
 */
int lc709204f_init_interrupt(const struct device *dev)
{
    struct lc709204f_data *data = dev->data;
    const struct lc709204f_config *config = dev->config;
    int err;

    if (!device_is_ready(config->gpio_int.port)) {
        LOG_ERR("nint_gpio device is not ready");
        return -ENODEV;
    }


    k_mutex_init(&data->threshold_mutex);

#if defined(CONFIG_LC709204F_TRIGGER_OWN_THREAD)
    k_sem_init(&data->gpio_sem, 0, K_SEM_MAX_LIMIT);

    k_thread_create(&data->thread, data->thread_stack,
            CONFIG_LC709204F_THREAD_STACK_SIZE,
            (k_thread_entry_t)lc709204f_thread, data, NULL, NULL, 
            K_PRIO_COOP(CONFIG_LC709204F_THREAD_PRIORITY), 0, K_NO_WAIT);
#elif defined(CONFIG_LC709204F_TRIGGER_GLOBAL_THREAD)
    data->work.handler = lc709204f_work_cb;
#endif /* CONFIG_LC709204F_TRIGGER_OWN_THREAD */

    err = gpio_pin_configure_dt(&config->gpio_int, GPIO_INPUT);
    if (err < 0) {
        LOG_DBG("Could not configure gpio");
        return err;
    }

    gpio_init_callback(&data->gpio_cb, lc709204f_gpio_callback, 
            BIT(config->gpio_int.pin));

    if (gpio_add_callback(config->gpio_int.port, &data->gpio_cb) < 0) {
        LOG_DBG("Could not set gpio callback");
        return -EIO;
    }

    return gpio_pin_interrupt_configure_dt(&config->gpio_int, 
            GPIO_INT_EDGE_TO_ACTIVE);
}