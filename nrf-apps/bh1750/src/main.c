#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>


void main(void){
	mode = CHM;
  	measuringTimeFactor = 1;
  	setMode();
  	setMeasuringTime(); 
  	k_msleep(200);

	if (!device_is_ready(dev)) {
		printk("sensor: device not ready.\n");
		return;
	}

	const struct device *dev1 = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	if (usb_enable(NULL)) {
		return;
	}

	/* Poll if the DTR flag was set */
	while (!dtr) {
		uart_line_ctrl_get(dev1, UART_LINE_CTRL_DTR, &dtr);
		/* Give CPU resources to low priority threads. */
		k_sleep(K_MSEC(100));
	}

	while(1){
		getLux();
		k_msleep(100);
	}
}

