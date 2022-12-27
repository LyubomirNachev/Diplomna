#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000


void main(void)
{
	while (1) {
		k_msleep(SLEEP_TIME_MS);
	}
}
// #include <zephyr/zephyr.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/usb/usb_device.h>
// #include <zephyr/drivers/uart.h>

// void main(void)
// {
// 	const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
// 	uint32_t dtr = 0;

// 	if (usb_enable(NULL)) {
// 		return;
// 	}

// 	/* Poll if the DTR flag was set */
// 	while (!dtr) {
// 		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
// 		/* Give CPU resources to low priority threads. */
// 		k_sleep(K_MSEC(100));
// 	}

// 	while (1) {
// 		printk("Hello World! %s\n", CONFIG_ARCH);
// 		k_sleep(K_SECONDS(1));
// 	}
// }
