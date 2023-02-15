#define DT_DRV_COMPAT rohm_bh1750

#include <zephyr/drivers/i2c.h>
#include "bh1750.h"

float measuringTimeFactor = 1;
uint8_t i2c_buffer[2];

static const struct device *dev = DEVICE_DT_GET(I2C_NODE);
int err;

extern int getLux(){
  if (!device_is_ready(dev)){
    printk("device is not ready\n");
    return 0;
  }
	uint16_t raw_lux;
	float lux;
	raw_lux = readBH1750();
	if((BH1750_MODE == 0b00010001)||(BH1750_MODE == 0b00100001)){
		lux = (raw_lux/2.4)/measuringTimeFactor;
	}else{
	  lux = (raw_lux/1.2)/measuringTimeFactor;
	}
  return (int)lux;
}

extern void powerDown(){
	writeBH1750(BH1750_POWER_DOWN);
}


void setMode(){
  writeBH1750(BH1750_MODE);
}

extern void powerOn(){
	writeBH1750(BH1750_POWER_ON);
	setMode();
}

extern void dataRegReset(){
  writeBH1750(BH1750_DATA_REG_RESET);
}


extern void setMeasuringTime(){
  uint8_t mt = measuringTimeFactor*69; // implementirai nachin za promqna na measuring time factora i promqna na MTReg (napravi kconfig opciq)
  uint8_t highByteMT = ((mt>>5) | 0b01000000);
  uint8_t lowByteMT = (mt & 0b01111111);
  lowByteMT |= 0b01100000;
  writeBH1750(highByteMT);
  writeBH1750(lowByteMT);
}

extern uint16_t readBH1750(){
  err = i2c_read(dev, i2c_buffer, 2, BH1750_ADDRESS_1);
  // if (err < 0){
  //   printk("Read Failed: %d\n", err);
  // }
  return ((i2c_buffer[0]<<8) + i2c_buffer[1]);
}


extern void writeBH1750(uint8_t val){
  i2c_buffer[0] = val;
  // printk("value: %d\n", i2c_buffer[0]);
    err = i2c_write(dev, i2c_buffer, 1, BH1750_ADDRESS_1);
    // if (err < 0){
    //   printk("Write Failed: %d\n", err);
    // }
}


