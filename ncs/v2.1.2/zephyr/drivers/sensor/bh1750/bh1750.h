#ifndef ZEPHYR_DRIVERS_SENSOR_BH1750_BH1750_H_
#define ZEPHYR_DRIVERS_SENSOR_BH1750_BH1750_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#define BH1750_ADDRESS_1 0x23
#define BH1750_ADDRESS_2 0x5C
#define BH1750_DATA_REG_RESET 0b00000111
#define BH1750_POWER_DOWN 0b00000000
#define BH1750_POWER_ON 0b00000001

#if CONFIG_BH1750_MODE_CHM
	#define BH1750_MODE 0b00010000
#elif CONFIG_BH1750_MODE_CHM_2
	#define BH1750_MODE 0b00010001
#elif CONFIG_BH1750_MODE_CLM
	#define BH1750_MODE 0b00010011
#elif CONFIG_BH1750_MODE_OTH
	#define BH1750_MODE 0b00100000
#elif CONFIG_BH1750_MODE_OTH_2
	#define BH1750_MODE 0b00100001
#elif CONFIG_BH1750_MODE_OTL
	#define BH1750_MODE 0b00100011
#endif

#define MEASUREMENT_TIME_REGISTER 

extern void getLux();
extern void powerDown();
extern void powerOn();
extern void dataRegReset();
extern void setMode();
extern void setMeasuringTime();
extern uint16_t readBH1750();
extern void writeBH1750(uint8_t val);

#endif /* ZEPHYR_DRIVERS_SENSOR_BH1750_BH1750_H_ */
