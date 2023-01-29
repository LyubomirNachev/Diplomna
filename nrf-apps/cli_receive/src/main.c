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

LOG_MODULE_REGISTER(cli_sample, CONFIG_OT_COMMAND_LINE_INTERFACE_LOG_LEVEL);

void udpReceiveCb(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo){
	uint16_t payloadLength = otMessageGetLength(aMessage) - otMessageGetOffset(aMessage);
	char buf[payloadLength+1];
	otMessageRead(aMessage, otMessageGetOffset(aMessage), buf, payloadLength);
	buf[payloadLength]='\0';
	//printk("Received: %s\n", buf);
	LOG_INF("Received: %s\n", buf);
}

static void udp_init(void){
	otError error = OT_ERROR_NONE;
	otInstance *myInstance = openthread_get_default_instance();
	otUdpSocket mySocket;
	otSockAddr mySockAddr;
	memset(&mySockAddr, 0, sizeof(mySockAddr));
	mySockAddr.mPort = 49155;

	do{
		error = otUdpOpen(myInstance, &mySocket, udpReceiveCb, NULL);
		if (error != OT_ERROR_NONE){break;}
		error = otUdpBind(myInstance, &mySocket, &mySockAddr, OT_NETIF_THREAD);
	}while(false);

	if(error != OT_ERROR_NONE){
		LOG_INF("init_udp error: %d\n", error);
	}
}



void main(void)
{
	udp_init();

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

	/* Data Carrier Detect Modem - mark connection as established */
	(void)uart_line_ctrl_set(dev, UART_LINE_CTRL_DCD, 1);
	/* Data Set Ready - the NCP SoC is ready to communicate */
	(void)uart_line_ctrl_set(dev, UART_LINE_CTRL_DSR, 1);
}


