#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>

#include <drivers/i2c.h>
#include <sys/crc.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(test_app);


static inline float out_ev(struct sensor_value *val)
{
	return (val->val1 + (float)val->val2 / 1000000);
}

static void fetch_and_display_lsm(const struct device *dev)
{
	struct sensor_value x, y, z;

    if (dev == NULL) {
        return;
    }
	/* lsm6dso accel */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &z);

	LOG_INF("accel x:%f ms/2 y:%f ms/2 z:%f ms/2",
			out_ev(&x), out_ev(&y), out_ev(&z));

	/* lsm6dso gyro */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &z);

	LOG_INF("gyro x:%f dps y:%f dps z:%f dps",
			out_ev(&x), out_ev(&y), out_ev(&z));
}


static int set_sampling_freq(const struct device *dev)
{
	int ret = 0;
	struct sensor_value odr_attr;

    if (dev == NULL) {
        return -ENODEV;
    }

	/* set accel/gyro sampling frequency to 12.5 Hz */
	odr_attr.val1 = 12.5;
	odr_attr.val2 = 0;

	ret = sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (ret != 0) {
		LOG_ERR("Cannot set sampling frequency for accelerometer.\n");
		return ret;
	}

	ret = sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (ret != 0) {
		LOG_ERR("Cannot set sampling frequency for gyro.\n");
		return ret;
	}

	return 0;
}

static void fetch_and_display_bme(const struct device *dev)
{
    struct sensor_value temp, press, humidity, gas_res;

    if (dev == NULL) {
        return;
    }

    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
    sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);
    sensor_channel_get(dev, SENSOR_CHAN_GAS_RES, &gas_res);

    LOG_INF("T: %f; P: %f; H: %f; G: %d", out_ev(&temp), out_ev(&press),
            out_ev(&humidity), gas_res.val1);

}

static void fetch_and_display_si(const struct device *dev)
{
    struct sensor_value light, ir, uv;

    if (dev == NULL) {
        return;
    }
    
    sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);

    sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &light);
    sensor_channel_get(dev, SENSOR_CHAN_IR, &ir);
    sensor_channel_get(dev, SENSOR_CHAN_RED, &uv);

    LOG_INF("light: %d, ir: %d, uv: %f", light.val1, ir.val1, out_ev(&uv));

}

static void fetch_and_display_lc(const struct device *dev)
{
    struct sensor_value voltage, capacity, soh, soc, time;

    if (dev == NULL) {
        return;
    }

    sensor_sample_fetch(dev);

    sensor_channel_get(dev, SENSOR_CHAN_GAUGE_VOLTAGE, &voltage);
    sensor_channel_get(dev, SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY, &capacity);
    sensor_channel_get(dev, SENSOR_CHAN_GAUGE_STATE_OF_HEALTH, &soh);
    sensor_channel_get(dev, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &soc);
    sensor_channel_get(dev, SENSOR_CHAN_GAUGE_TIME_TO_EMPTY, &time);

    LOG_INF("volt: %f, capacity: %d, SoH: %d%%, SoC: %f%%, TTE: %d mins", 
            out_ev(&voltage), capacity.val1, soh.val1, out_ev(&soc), time.val1);
}


void main(void)
{
    const struct device *bme680 = device_get_binding(DT_LABEL(
            DT_INST(0, bosch_bme680)));

    const struct device *lsm6dso = device_get_binding(DT_LABEL(
            DT_INST(0, st_lsm6dso)));

    const struct device *si1132 = device_get_binding(DT_LABEL(
            DT_INST(0, silabs_si1132)));

    const struct device *lc709204f = device_get_binding(DT_LABEL(
            DT_INST(0, onnn_lc709204f)));

    
    if (bme680 != NULL)
        LOG_WRN("Device %p name is %s", bme680, bme680->name);
    
    if (lsm6dso != NULL)
        LOG_WRN("Device %p name is %s", lsm6dso, lsm6dso->name);
    
    if (si1132 != NULL)
        LOG_WRN("Device %p name is %s", si1132, si1132->name);

    if (lc709204f != NULL)
        LOG_WRN("Device %p name is %s", lc709204f, lc709204f->name);

    if (set_sampling_freq(lsm6dso) != 0) {
        LOG_ERR("Unable to set sampling frequency of LSM6DSO");
    }


    while (1) {
        k_sleep(K_MSEC(3000));

        fetch_and_display_lsm(lsm6dso);

        fetch_and_display_bme(bme680);

        fetch_and_display_si(si1132);

        fetch_and_display_lc(lc709204f);

        LOG_INF("\n");
        
    }
}