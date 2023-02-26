#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <openthread/thread.h>
#include <net/openthread.h>

#include <zephyr/logging/log.h>
#include <openthread/udp.h>

LOG_MODULE_REGISTER(cli_sample, CONFIG_OT_COMMAND_LINE_INTERFACE_LOG_LEVEL);

static const struct gpio_dt_spec button0_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(button0), gpios);
static struct gpio_callback button0_cb;

static void udp_send(void){
	otError	error = OT_ERROR_NONE;
	const char *buf = "Hello Thread"; 
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
		LOG_INF("Send.\n"); 
	}else{
		LOG_INF("udpSend error: %d\n", error); 
	}
}

void button_pressed_callback(const struct device *gpiob, struct gpio_callback *cb, gpio_port_pins_t pins){ 
	udp_send();
}

void main(void)
{
	gpio_pin_configure_dt(&button0_spec, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button0_cb, button_pressed_callback, BIT(button0_spec.pin));
	gpio_add_callback(button0_spec.port, &button0_cb);


	otInstance *myInstance;
	myInstance = openthread_get_default_instance();
	otPlatRadioSetTransmitPower(myInstance, 8);
	int err = usb_enable(NULL);
	const struct device *dev;
	uint32_t dtr = 0;
 
	if (err != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}

	dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
	if (dev == NULL) {
		LOG_ERR("Failed to find specific UART device");
		return;
	}

	/* Data Terminal Ready - check if host is ready to communicate */
	while (!dtr) {
		err = uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		if (err) {
			LOG_ERR("Failed to get Data Terminal Ready line state: %d", err);
			continue;
		}
		k_msleep(100);
	}
}



















// static void udp_send(void){
// 	otError			error = OT_ERROR_NONE;
// 	const char *buf = "Hello Thread"; // буфер от символи, съдържащ данните, които ще бъдат изпратени

// 	otInstance *myInstance; // пойнтер към дефинираната OpenThread инстанция
// 	myInstance = openthread_get_default_instance(); // задаване на основната Openthread инстанция
// 	otUdpSocket mySocket; // структура, репрезентираща UDP цокълът

// 	otMessageInfo messageInfo; // структура, репрезентираща локалният и peer IPv6 адрес на цокъла
// 	memset(&messageInfo, 0, sizeof(messageInfo)); // подсигуряване, че messageInfo структурата ще бъде празна
// 	otIp6AddressFromString("ff03::1", &messageInfo.mPeerAddr); // преобразуване на IPv6 адреса в двоичен формат
// 	messageInfo.mPeerPort = 12346; // използваният udp port

// 	do{
// 		error = otUdpOpen(myInstance, &mySocket, NULL, NULL); //отваряне на UDP цокъла
// 		if (error != OT_ERROR_NONE){ break; }
// 		otMessage *test_Message = otUdpNewMessage(myInstance, NULL); // създаване на UDP съобщение

// 		// добавяне на байтовете от буфера съдържащ информацията към otMessage
// 		error = otMessageAppend(test_Message, buf, (uint16_t)strlen(buf));  
// 		if (error != OT_ERROR_NONE){ break; }
// 		error = otUdpSend(myInstance, &mySocket, test_Message, &messageInfo); // изпращане на otMessage съобщението
// 		if (error != OT_ERROR_NONE){ break; }
// 		error = otUdpClose(myInstance, &mySocket); // затваряне на UDP цокъла
// 	}while(false); // цикълът подсигурява, че при грешка изпълнението ще бъде прекъснато
	
// 	if(error == OT_ERROR_NONE){
// 		LOG_INF("Send.\n"); //съобщение за успешно изпращане
// 	}else{
// 		LOG_INF("udpSend error: %d\n", error); //съобщение за грешка
// 	}
// }

// // функция, проверяваща дали е натиснат бутона
// void button_pressed_callback(const struct device *gpiob, struct gpio_callback *cb, gpio_port_pins_t pins){ 
// 	udp_send(); //изпращане на данни 
// }

// void main(void)
// {
// 	// конфигурация на бутона 
// 	gpio_pin_configure_dt(&button0_spec, GPIO_INPUT);
// 	gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_EDGE_TO_ACTIVE);
// 	gpio_init_callback(&button0_cb, button_pressed_callback, BIT(button0_spec.pin));
// 	gpio_add_callback(button0_spec.port, &button0_cb);
// 	//

// 	int err = usb_enable(NULL); // 
// 	const struct device *dev;
// 	uint32_t dtr = 0;
 
// 	if (err != 0) {
// 		LOG_ERR("Failed to enable USB");
// 		return;
// 	}

// 	dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
// 	if (dev == NULL) {
// 		LOG_ERR("Failed to find specific UART device");
// 		return;
// 	}

// 	LOG_INF("Waiting for host to be ready to communicate");

// 	/* Data Terminal Ready - check if host is ready to communicate */
// 	while (!dtr) {
// 		err = uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
// 		if (err) {
// 			LOG_ERR("Failed to get Data Terminal Ready line state: %d", err);
// 			continue;
// 		}
// 		k_msleep(100);
// 	}

// }

