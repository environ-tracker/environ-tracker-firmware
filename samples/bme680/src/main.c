/*
 * Copyright (c) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(BME680);

void main(void)
{
	const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, bosch_bme680)));
	struct sensor_value temp, press, humidity, gas_res;

	LOG_WRN("Device %p name is %s\n", dev, dev->name);

	while (1) {
		k_sleep(K_MSEC(3000));

		LOG_WRN("Running loop");


		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);
		sensor_channel_get(dev, SENSOR_CHAN_GAS_RES, &gas_res);

		LOG_WRN("T: %d.%06d; P: %d.%06d; H: %d.%06d; G: %d.%06d\n",
				temp.val1, temp.val2, press.val1, press.val2,
				humidity.val1, humidity.val2, gas_res.val1,
				gas_res.val2);
	}
}
