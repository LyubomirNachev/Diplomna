#include <stdio.h>
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <zephyr/logging/log.h>
#include <openthread/thread.h>
#include <net/openthread.h>
#include <openthread/udp.h>
#include <zephyr/drivers/sensor.h>

LOG_MODULE_REGISTER(cli_sample, CONFIG_OT_COMMAND_LINE_INTERFACE_LOG_LEVEL);

static void udp_send(int light, int temp1, int temp2, int hum1, int hum2, int press1, int press2, int gas1, int gas2){
	otError	error = OT_ERROR_NONE;
	char data[300];
	sprintf(data, "Light: %dlx Temp: %d.%d°C Humidity: %d.%d%% Pressure: %d.%dhPa Gas: %d.%dΩ", light, temp1, temp2, hum1, hum2, press1, press2, gas1, gas2);
	otInstance *myInstance;
	myInstance = openthread_get_default_instance();
	otUdpSocket mySocket;

	otMessageInfo messageInfo;
	memset(&messageInfo, 0, sizeof(messageInfo));
	otIp6AddressFromString("ff03::1", &messageInfo.mPeerAddr);
	messageInfo.mPeerPort = 12346;

	do{
		error = otUdpOpen(myInstance, &mySocket, NULL, NULL);
		if (error != OT_ERROR_NONE){ break; }
		otMessage *test_Message = otUdpNewMessage(myInstance, NULL);
		error = otMessageAppend(test_Message, data, (uint16_t)strlen(data));
		if (error != OT_ERROR_NONE){ break; }
		error = otUdpSend(myInstance, &mySocket, test_Message, &messageInfo);
		if (error != OT_ERROR_NONE){ break; }
		error = otUdpClose(myInstance, &mySocket);
	}while(false);
	
	if(error == OT_ERROR_NONE){
		LOG_INF("Send. %s\n", data);
	}else{
		LOG_INF("udpSend error: %d\n", error);
	}
}

void main(void)
{
	int ret;
	const struct device *dev;

	powerOn();
    setMeasuringTime(); 

	otInstance *myInstance;
	myInstance = openthread_get_default_instance();
	otPlatRadioSetTransmitPower(myInstance, 8);

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

	const struct device *const dev_bme = DEVICE_DT_GET(DT_NODELABEL(bme688_sensor));
	struct sensor_value temp, press, humidity, gas_res;

	while(1){

		sensor_sample_fetch(dev_bme);
		sensor_channel_get(dev_bme, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev_bme, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(dev_bme, SENSOR_CHAN_HUMIDITY, &humidity);
		sensor_channel_get(dev_bme, SENSOR_CHAN_GAS_RES, &gas_res);

		udp_send(getLux(), temp.val1, temp.val2, humidity.val1, humidity.val2, press.val1, press.val2, gas_res.val1, gas_res.val2);
		k_sleep(K_MSEC(1000));
	}
}

