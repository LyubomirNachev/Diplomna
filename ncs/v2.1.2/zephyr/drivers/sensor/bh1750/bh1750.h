#ifndef ZEPHYR_DRIVERS_SENSOR_BH1750_BH1750_H_
#define ZEPHYR_DRIVERS_SENSOR_BH1750_BH1750_H_

#include <zephyr/device.h>
#include <zephyr/devicetree.h> 
#include <zephyr/kernel.h>
//#include <zephyr/drivers/sensor.h>
//#include <zephyr/drivers/i2c.h>
//#include <zephyr/drivers/gpio.h>

#define BH1750_ADDRESS_1         0x23
#define BH1750_ADDRESS_2         0x5C
#define BH1750_DATA_REG_RESET    0b00000111
#define BH1750_POWER_DOWN        0b00000000
#define BH1750_POWER_ON          0b00000001
#define I2C_NODE                 DT_NODELABEL(i2c0) 

enum BH1750Mode {
  CHM = 0b00010000,     //CHM: Continuously H-Resolution Mode
  CHM_2 = 0b00010001,   //CHM_2: Continuously H-Resolution Mode2
  CLM = 0b00010011,     //CLM: Continuously L-Resolution Mode
  OTH = 0b00100000,     //OTH: One Time H-Resolution Mode
  OTH_2 = 0b00100001,   //OTH_2: One Time H-Resolution Mode2
  OTL = 0b00100011      //OTL: One Time L-Resolution Mode
} mode;

extern void getLux();
extern void powerDown();
extern void powerOn();
extern void dataRegReset();
extern void setMode();
extern void setMeasuringTime();
extern uint16_t readBH1750();
extern void writeBH1750(uint8_t val);

#endif /* ZEPHYR_DRIVERS_SENSOR_BH1750_BH1750_H_ */
