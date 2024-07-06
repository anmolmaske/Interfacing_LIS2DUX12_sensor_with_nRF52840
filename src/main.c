/**
* \file main.c
*
* \brief LIS2DUX12 Sensor Interfacing
*
* \date 2024-07-04
*/

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(LIS2DUX, CONFIG_LOG_DEFAULT_LEVEL);

static void fetch_and_display(const struct device *sensor)
{
	struct sensor_value accel[3];

	int rc = sensor_sample_fetch(sensor);
    if (rc != 0) {
        LOG_INF("ERROR: Sensor sample fetch failed: %d\n", rc);
        return;
    }

	rc = sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ, accel);
    if (rc != 0) {
        LOG_INF("ERROR: Sensor channel get failed: %d\n", rc);
    } else {
        LOG_INF("X: %.2f,   Y: %.2f,   Z: %.2f",
               sensor_value_to_double(&accel[0]),
               sensor_value_to_double(&accel[1]),
               sensor_value_to_double(&accel[2]));
    }
}


int main(void)
{
	const struct device *const sensor = DEVICE_DT_GET_ANY(st_lis2dux12);

	if (sensor == NULL) {
		LOG_INF("No device found\n");
		return 0;
	}
	if (!device_is_ready(sensor)) {
		LOG_INF("Device %s is not ready\n", sensor->name);
		return 0;
	}

	k_sleep(K_MSEC(1000));

	LOG_INF("Acceleration measured are as follows:");

	while (true) 
	{
		fetch_and_display(sensor);
		k_sleep(K_MSEC(1000));
	}
}
