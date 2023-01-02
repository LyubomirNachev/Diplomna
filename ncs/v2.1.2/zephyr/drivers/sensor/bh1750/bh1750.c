#define DT_DRV_COMPAT rohm_bh1750

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include "bh1750.h"

LOG_MODULE_REGISTER(BH1750, CONFIG_SENSOR_LOG_LEVEL);

float measuringTimeFactor;

static const struct device *dev = DEVICE_DT_GET(I2C_NODE);
static uint8_t i2c_buffer[2];
int err;

void getLux(){
	uint16_t raw_lux;
	float lux;
	raw_lux = readBH1750();
	if((mode==CHM_2)||(mode==OTH_2)){
		lux = (raw_lux/2.4)/measuringTimeFactor;
	}else{
		lux = (raw_lux/1.2)/measuringTimeFactor;
	}
	printk("Light: %flx", lux);
}

void powerDown(){
	writeBH1750(BH1750_POWER_DOWN);
}

void powerOn(){
	writeBH1750(BH1750_POWER_ON);
	setMode();
}

void dataRegReset(){
  writeBH1750(BH1750_DATA_REG_RESET);
}

void setMode(){
  writeBH1750(mode);
}

void setMeasuringTime(){
  uint8_t mt = round(measuringTimeFactor*69);
  uint8_t highByteMT = ((mt>>5) | 0b01000000);
  uint8_t lowByteMT = (mt & 0b01111111);
  lowByteMT |= 0b01100000;
  writeBH1750(highByteMT);
  writeBH1750(lowByteMT);
}

uint16_t readBH1750(){
  uint8_t MSbyte, LSbyte;
  do{
    err = i2c_read(dev, i2c_buffer, 2, BH1750_ADDRESS_1);
    if (err < 0){
      printk("Read Failed: %d\n", err);
      break;
    }
  }while(false);
  MSbyte = i2c_buffer[0];
  LSbyte = i2c_buffer[1];
  return ((MSbyte<<8) + LSbyte);
}

void writeBH1750(uint8_t val){
  do{
    err = i2c_write(dev, i2c_buffer, 1, BH1750_ADDRESS_1);
    if (err < 0){
      printk("Write Failed: %d\n", err);
      break;
    }
  }while(false);
}