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
#include <inttypes.h>

#include <zephyr/drivers/adc.h>


//Battery
#define ADC_NODE DT_NODELABEL(adc)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

#define ADC_RESOLUTION		10
#define ADC_GAIN			ADC_GAIN_1_5
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_PORT SAADC_CH_PSELP_PSELP_AnalogInput0
#define ADC_CHANNEL 0

struct adc_channel_cfg chl0_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_CHANNEL,
#ifdef CONFIG_ADC_NRFX_SAADC
	.input_positive = ADC_PORT
#endif
};  

int16_t sample_buffer[1];
struct adc_sequence sequence = {
	.channels = BIT(ADC_CHANNEL),
	.buffer = sample_buffer,
	.buffer_size = sizeof(sample_buffer),
	.resolution = ADC_RESOLUTION
};
//Battery

LOG_MODULE_REGISTER(cli_sample, CONFIG_OT_COMMAND_LINE_INTERFACE_LOG_LEVEL);

#define BUTTON0_NODE DT_NODELABEL(button0)
static const struct gpio_dt_spec button0_spec = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);
static struct gpio_callback button0_cb;


static void udp_send(int light, int temp1, int temp2, int hum1, int hum2, int press1, int press2, int gas1, int gas2, float bat_lvl){
	otError			error = OT_ERROR_NONE;
	char data[300];
	sprintf(data, "Light: %dlx Temp: %d.%d°C Humidity: %d.%d%% Pressure: %d.%dhPa Gas: %d.%dΩ Battery: %dmV", light, temp1, temp2, hum1, hum2, press1, press2, gas1, gas2, bat_lvl);
	char *buf = &data;
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

	otInstance *myInstance;
	myInstance = openthread_get_default_instance();
	otPlatRadioSetTransmitPower(myInstance, 8);
	
	int ret;
	const struct device *dev;
	uint32_t dtr = 0U;
	int err = adc_channel_setup(adc_dev, &chl0_cfg);

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

	const struct device *const dev_bme = DEVICE_DT_GET(DT_NODELABEL(bme688_sensor));
	struct sensor_value temp, press, humidity, gas_res;

	while(1){

		sensor_sample_fetch(dev_bme);
		sensor_channel_get(dev_bme, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev_bme, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(dev_bme, SENSOR_CHAN_HUMIDITY, &humidity);
		sensor_channel_get(dev_bme, SENSOR_CHAN_GAS_RES, &gas_res);
		//LOG_INF("T: %d.%06d; P: %d.%06d; H: %d.%06d; G: %d.%06d\n",temp.val1, temp.val2, press.val1, press.val2,humidity.val1, humidity.val2, gas_res.val1,gas_res.val2);
		err = adc_read(adc_dev, &sequence);
		//LOG_INF("ADC: %d\n", sample_buffer[0]);
		int32_t mv_value = sample_buffer[0];
		int32_t adc_vref = adc_ref_internal(adc_dev);
		adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &mv_value);
		int32_t current_consumption = (int32_t)((mv_value/100000.0)*1000000.0);
		//LOG_INF("ADC converted: %d mV; %d uA\n", mv_value, current_consumption);
		

		udp_send(getLux(), temp.val1, temp.val2, humidity.val1, humidity.val2, press.val1, press.val2, gas_res.val1, gas_res.val2, mv_value);

		k_sleep(K_MSEC(1000));
	}






	/* Data Carrier Detect Modem - mark connection as established */
	(void)uart_line_ctrl_set(dev, UART_LINE_CTRL_DCD, 1);
	/* Data Set Ready - the NCP SoC is ready to communicate */
	(void)uart_line_ctrl_set(dev, UART_LINE_CTRL_DSR, 1);

}

