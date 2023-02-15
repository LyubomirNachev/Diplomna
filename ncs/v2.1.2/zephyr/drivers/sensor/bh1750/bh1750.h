#ifndef ZEPHYR_DRIVERS_SENSOR_BH1750_BH1750_H_
#define ZEPHYR_DRIVERS_SENSOR_BH1750_BH1750_H_

#include <zephyr/device.h>
#include <zephyr/devicetree.h> 
#include <zephyr/kernel.h>

#define BH1750_ADDRESS_1         0x23
#define BH1750_ADDRESS_2         0x5C
#define BH1750_DATA_REG_RESET    0x07
#define BH1750_POWER_DOWN        0x00
#define BH1750_POWER_ON          0x01
#define I2C_NODE                 DT_NODELABEL(i2c0) 







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

extern int getLux();
extern void powerDown();
extern void powerOn();
extern void dataRegReset();
extern void setMeasuringTime();
extern uint16_t readBH1750();
extern void writeBH1750(uint8_t val);

#endif /* ZEPHYR_DRIVERS_SENSOR_BH1750_BH1750_H_ */








//#include <zephyr/drivers/sensor.h>
//#include <zephyr/drivers/i2c.h>
//#include <zephyr/drivers/gpio.h>






// static const enum BH1750Mode {
//   CHM = 0b00010000,     //CHM: Continuously H-Resolution Mode //0x10
//   CHM_2 = 0b00010001,   //CHM_2: Continuously H-Resolution Mode2 //0x11
//   CLM = 0b00010011,     //CLM: Continuously L-Resolution Mode //0x13
//   OTH = 0b00100000,     //OTH: One Time H-Resolution Mode //0x20
//   OTH_2 = 0b00100001,   //OTH_2: One Time H-Resolution Mode2 //0x21
//   OTL = 0b00100011      //OTL: One Time L-Resolution Mode //0x23
// }mode;