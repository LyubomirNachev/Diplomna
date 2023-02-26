#include <stdio.h>
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>
#include <sys/printk.h>

void main(void){
	const struct device *dev = DEVICE_DT_GET_ANY(bosch_bme688);
	const struct device *dev_console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    struct sensor_value temp, press, humidity, gas_res;
    uint32_t dtr = 0;

    if (usb_enable(NULL)){
		return;
	}

	/* Poll if the DTR flag was set */
	while (!dtr) {
		uart_line_ctrl_get(dev_console, UART_LINE_CTRL_DTR, &dtr);
		/* Give CPU resources to low priority threads. */
		k_sleep(K_MSEC(100));
	}

	if (dev == NULL) {
		printk("\nError: no device found.\n");
	}

    while (1) {
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);
		sensor_channel_get(dev, SENSOR_CHAN_GAS_RES, &gas_res);

		printf("T: %d.%06d; P: %d.%06d; H: %d.%06d; G: %d.%06d\n",
				temp.val1, temp.val2, press.val1, press.val2,
				humidity.val1, humidity.val2, gas_res.val1,
				gas_res.val2);
        k_sleep(K_MSEC(10000));
    }
}