#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <openthread/thread.h>
#include <net/openthread.h>
#include <openthread/udp.h>
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>



LOG_MODULE_REGISTER(cli_sample, CONFIG_OT_COMMAND_LINE_INTERFACE_LOG_LEVEL);

#define BUTTON0_NODE DT_NODELABEL(button0)
static const struct gpio_dt_spec button0_spec = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);
static struct gpio_callback button0_cb;

static void udp_send(int val1){
	otError			error = OT_ERROR_NONE;
	char data[30];
	sprintf(data, "%d", val1);
	char *buf = &data;
	otInstance *myInstance;
	myInstance = openthread_get_default_instance();
	otUdpSocket mySocket;

	otMessageInfo messageInfo;
	memset(&messageInfo, 0, sizeof(messageInfo));
	otIp6AddressFromString("ff02::1", &messageInfo.mPeerAddr);
	messageInfo.mPeerPort = 12346;

	do{
		error = otUdpOpen(myInstance, &mySocket, NULL, NULL);
		if (error != OT_ERROR_NONE){ break; }
		otMessage *test_Message = otUdpNewMessage(myInstance, NULL);
		error = otMessageAppend(test_Message, buf, (uint16_t)strlen(buf));
		if (error != OT_ERROR_NONE){ break; }
		error = otUdpSend(myInstance, &mySocket, test_Message, &messageInfo);
		if (error != OT_ERROR_NONE){ break; }
		error = otUdpClose(myInstance, &mySocket);
	}while(false);
	
	if(error == OT_ERROR_NONE){
		LOG_INF("Send. %s\n", buf);
	}else{
		LOG_INF("udpSend error: %d\n", error);
	}
}

void button_pressed_callback(const struct device *gpiob, struct gpio_callback *cb, gpio_port_pins_t pins){
	
}


void main(void)
{
	powerOn();
    setMeasuringTime(); 

	gpio_pin_configure_dt(&button0_spec, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button0_cb, button_pressed_callback, BIT(button0_spec.pin));
	gpio_add_callback(button0_spec.port, &button0_cb);


	otPlatRadioSetTransmitPower(NULL, 8);
	int ret;
	const struct device *dev;
	uint32_t dtr = 0U;

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}

	dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
	if (dev == NULL) {
		LOG_ERR("Failed to find specific UART device");
		return;
	}

	LOG_INF("Waiting for host to be ready to communicate");

	/* Data Terminal Ready - check if host is ready to communicate */
	while (!dtr) {
		ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		if (ret) {
			LOG_ERR("Failed to get Data Terminal Ready line state: %d",
				ret);
			continue;
		}
		k_msleep(100);
	}
	while(1){
		udp_send(getLux());
		k_sleep(K_MSEC(100));
	}

	/* Data Carrier Detect Modem - mark connection as established */
	(void)uart_line_ctrl_set(dev, UART_LINE_CTRL_DCD, 1);
	/* Data Set Ready - the NCP SoC is ready to communicate */
	(void)uart_line_ctrl_set(dev, UART_LINE_CTRL_DSR, 1);

}

