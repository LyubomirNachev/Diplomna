#include <stdio.h>
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <zephyr/logging/log.h>
#include <openthread/thread.h>
#include <net/openthread.h>
#include <openthread/udp.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>


//led
static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios,
						     {0});
static struct gpio_dt_spec power_on_led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1_blue), gpios,
						     {0});
int power_on_val = 1;
int val = 0;
//led

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

static void udp_send(int light, int temp1, int temp2, int hum1, int hum2,
 					 int press1, int press2, int gas1, int gas2, char* moved, int bat_lvl){
	otError	error = OT_ERROR_NONE;
	char data[300];
	sprintf(data, "ID:%08X%08X Light: %dlx Temp: %d.%d°C Humidity: %d.%d%% Pressure: %d.%dhPa Gas: %d.%dΩ Moved: %s Battery: %dmV", 
				    NRF_FICR->DEVICEID[0], NRF_FICR->DEVICEID[1], light, temp1, temp2, hum1, hum2, press1, press2, gas1, gas2, moved, bat_lvl);
	otInstance *myInstance;
	myInstance = openthread_get_default_instance();
	otUdpSocket mySocket;

	otMessageInfo myInfo;

	//Making sure that the myInfo structure is empty
	memset(&myInfo, 0, sizeof(myInfo)); 

	//Set the address to which the data will be send
	otIp6AddressFromString("ff03::1", &myInfo.mPeerAddr); 

	myInfo.mPeerPort = 12346;

	// One cycle loop to enable the use of breaks
	do{ 
		error = otUdpOpen(myInstance, &mySocket, NULL, NULL);
		if (error != OT_ERROR_NONE){ break; }
		otMessage *test_Message = otUdpNewMessage(myInstance, NULL);
		error = otMessageAppend(test_Message, data, (uint16_t)strlen(data));
		if (error != OT_ERROR_NONE){ break; }
		error = otUdpSend(myInstance, &mySocket, test_Message, &myInfo);
		if (error != OT_ERROR_NONE){ break; }
		error = otUdpClose(myInstance, &mySocket);
	}while(false);
	
	if(error == OT_ERROR_NONE){
		gpio_pin_configure_dt(&led, GPIO_OUTPUT);
		val=1;
		gpio_pin_set_dt(&led, val);
		LOG_INF("Send. %s\n", data);
		k_sleep(K_MSEC(500));
		val=0;
		gpio_pin_set_dt(&led, val);
		//LOG_INF("DEVICEID0: %08X\n", NRF_FICR->DEVICEID[0]);
		//LOG_INF("DEVICEID0: %08X\n", NRF_FICR->DEVICEID[1]);
		
	}else{
		LOG_INF("udpSend error: %d\n", error);
	}
}

//lis3dh
#include <zephyr/drivers/i2c.h>
#define I2C_NODE                 DT_NODELABEL(i2c0) 
#define LIS3DH_ADDRESS         0x19
static const struct device *dev_lis3dh = DEVICE_DT_GET(I2C_NODE);
uint8_t lis_buffer[2];
#define STATUS_REG_AUX 0x07
#define OUT_ADC1_L	   0x08
#define OUT_ADC1_H	   0x09
#define OUT_ADC2_L	   0x0A
#define OUT_ADC2_H	   0x0B
#define OUT_ADC3_L	   0x0C
#define OUT_ADC3_H	   0x0D
#define WHO_AM_I       0x0F
#define CTRL_REG0      0x1E
#define TEMP_CFG_REG   0x1F
#define CTRL_REG1      0x20
#define CTRL_REG2      0x21
#define CTRL_REG3      0x22 
#define CTRL_REG4      0x23
#define CTRL_REG5      0x24
#define CTRL_REG6      0x25
#define REFERENCE      0x26
#define STATUS_REG     0x27
#define OUT_X_L        0x28
#define OUT_X_H        0x29
#define OUT_Y_L		   0x2A
#define OUT_Y_H 	   0x2B
#define OUT_Z_L		   0x2C
#define OUT_Z_H 	   0x2D
#define FIFO_CTRL_REG  0x2E
#define FIFO_SRC_REG   0x2F
#define INT1_CFG       0x30
#define INT1_SRC       0x31
#define INT1_THS       0x32
#define INT1_DURATION  0x33
#define INT2_CFG 	   0x34
#define INT2_SRC       0X35
#define INT2_THS       0x36
#define INT2_DURATION  0x37
#define CLICK_CFG      0x38
#define CLICK_SRC      0x39
#define CLICK_THS      0x3A
#define TIME_LIMIT     0x3B
#define TIME_LATENCY   0x3C
#define TIME_WINDOW    0x3D
#define ACT_THS        0x3E
#define ACT_DUR        0x3F


	

uint8_t read_LIS3DH(){
  i2c_read(dev_lis3dh, lis_buffer, 1, LIS3DH_ADDRESS);
  return lis_buffer[0];
}


void write_LIS3DH(uint8_t val){
  lis_buffer[0] = val;
  i2c_write(dev_lis3dh, lis_buffer, 1, LIS3DH_ADDRESS);
}

void write_internal_LIS3DH(uint8_t reg, uint8_t val){
	i2c_reg_write_byte(dev_lis3dh, LIS3DH_ADDRESS, reg, val);
}

uint8_t read_internal_LIS3DH(uint8_t reg){
	i2c_reg_read_byte(dev_lis3dh, LIS3DH_ADDRESS, reg, lis_buffer);
	return lis_buffer[0];
}

uint8_t intsrc;
int8_t xl,yl,zl,xh,yh,zh,stat;
void config_LIS3DH(void){
	write_internal_LIS3DH(CTRL_REG1, 0x57);
	write_internal_LIS3DH(CTRL_REG2, 0x09);
	write_internal_LIS3DH(CTRL_REG3, 0x40); //0x10 ZYXDA interrupt on INT1.
	write_internal_LIS3DH(CTRL_REG4, 0x00);
	write_internal_LIS3DH(CTRL_REG5, 0x08); // 0x04 4D enable: 4D detection is enabled on INT1 when 6D bit on INT1_CFG is set to 1
	write_internal_LIS3DH(REFERENCE, 0x05); //Reference value for Interrupt generation. Default value: 0000 0000
	write_internal_LIS3DH(INT1_CFG, 0x2A);
	write_internal_LIS3DH(INT1_THS, 0x10);
	write_internal_LIS3DH(INT1_DURATION, 0x03);
}

int16_t x;
int16_t y;
int16_t z;
char moved[4];

void read_LIS3DH_data(void){
	intsrc = read_internal_LIS3DH(INT1_SRC);
	xl = read_internal_LIS3DH(OUT_X_L);
	xh = read_internal_LIS3DH(OUT_X_H);
	yl = read_internal_LIS3DH(OUT_Y_L);
	yh = read_internal_LIS3DH(OUT_Y_H);
	zl = read_internal_LIS3DH(OUT_Z_L);
	zh = read_internal_LIS3DH(OUT_Z_H);
	stat = read_internal_LIS3DH(WHO_AM_I);

	x = xh;
	x <<= 8;
	x |= xl;
	x>>=6;

	y = yh;
	y <<= 8;
	y |= yl;
	y>>=6;

	z = zh;
	z <<= 8;
	z |= zl;
	z>>=6;

	intsrc >>= 6;
	if(intsrc == 1) memcpy(moved, "yes", strlen("yes"));
	else memcpy(moved, "no", strlen("no")+1);
	LOG_INF("X: %d Y: %d, Z: %d INFO: %d Moved: %s", x,y,z,stat,moved);
}
 //lis3dh

void main(void)
{
	gpio_pin_configure_dt(&power_on_led, GPIO_OUTPUT);
	gpio_pin_set_dt(&power_on_led, power_on_val);
	int ret;
	const struct device *dev;
	int adc = adc_channel_setup(adc_dev, &chl0_cfg);

	otInstance *myInstance;
	myInstance = openthread_get_default_instance();
	otPlatRadioSetTransmitPower(myInstance, 8); //Increase the transmit power to 8dBm

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}

	dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart)); //Initialize the Zephyr Shell
	if (dev == NULL) {
		LOG_ERR("Failed to find specific UART device");
		return;
	}
	LOG_INF("Waiting for host to be ready to communicate");

	// Start the BH1750 sensor 
	powerOn();
    setMeasuringTime(); 

	// Initialize the BME688 sensor
	const struct device *const dev_bme = DEVICE_DT_GET(DT_NODELABEL(bme688_sensor));

	struct sensor_value temp, press, humidity, gas_res;


	config_LIS3DH();
	while(1){
		// Get values from the BME688 sensor
		sensor_sample_fetch(dev_bme);
		sensor_channel_get(dev_bme, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev_bme, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(dev_bme, SENSOR_CHAN_HUMIDITY, &humidity);
		sensor_channel_get(dev_bme, SENSOR_CHAN_GAS_RES, &gas_res);
		temp.val2 = temp.val2/10000;
		humidity.val2 = humidity.val2/10000;
		press.val2 = press.val2/1000;
		adc = adc_read(adc_dev, &sequence);
		//LOG_INF("ADC: %d\n", sample_buffer[0]);
		int32_t mv_value = sample_buffer[0];
		int32_t adc_vref = adc_ref_internal(adc_dev);
		adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &mv_value);
		read_LIS3DH_data();
		udp_send(getLux(), temp.val1, temp.val2, humidity.val1, humidity.val2,
				 press.val1, press.val2, gas_res.val1, gas_res.val2, moved, mv_value);
		k_sleep(K_MSEC(3000));
	}
}