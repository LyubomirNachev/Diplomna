#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>



#include <usb/usb_device.h>
#include <drivers/uart.h>

#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct device *get_bmp280_device(void)
{
	//const struct device *dev = DEVICE_DT_GET_ANY(bosch_bmp280);
    const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, bosch_bmp280)));

	if (dev == NULL) {
		printk("\nError: no device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

void main(void)
{

	struct sensor_value temp, press, humidity;


	const struct device *dev_console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

    gpio_pin_configure_dt(&led, GPIO_OUTPUT);

	if (usb_enable(NULL)) {
		return;
	}

	/* Poll if the DTR flag was set */
	while (!dtr) {
		uart_line_ctrl_get(dev_console, UART_LINE_CTRL_DTR, &dtr);
		/* Give CPU resources to low priority threads. */
		k_sleep(K_MSEC(100));
	}






	const struct device *dev = get_bmp280_device();
	if (dev == NULL) {
        printk("Eroorasdfsd");
		return;
	}
    int val = 0;
	while (1) {
		gpio_pin_set_dt(&led, val);
        if (val == 0) {
            val = 1;
        }else {
            val = 0;
        }


		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);

		printk("temp: %d.%06d; press: %d.%06d; humidity: %d.%06d\n",
		      temp.val1, temp.val2, press.val1, press.val2,
		      humidity.val1, humidity.val2);

       // printk("Hello World! %s\n", CONFIG_ARCH);
		k_sleep(K_MSEC(1000));
	}
}
