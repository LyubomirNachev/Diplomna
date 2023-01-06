/* bme688.c - Driver for Bosch Sensortec's BME688 temperature, pressure,
 * humidity and gas sensor
 *
 * https://www.bosch-sensortec.com/bst/products/all_products/bme688
 */

/*
 * Copyright (c) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bme688

#include "bme688.h"
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <init.h>
#include <kernel.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <drivers/sensor.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(bme688, CONFIG_SENSOR_LOG_LEVEL);

static int bme688_reg_read(const struct device *dev, uint8_t start,
			   uint8_t *buf, int size)
{
	const struct bme688_config *config = dev->config;

	return i2c_burst_read_dt(&config->bus, start, buf, size);
}

static int bme688_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct bme688_config *config = dev->config;

	return i2c_reg_write_byte_dt(&config->bus, reg, val);
}

static void bme688_calc_temp(struct bme688_data *data, uint32_t adc_temp)
{
	int64_t var1, var2, var3;

	var1 = ((int32_t)adc_temp >> 3) - ((int32_t)data->par_t1 << 1);
	var2 = (var1 * (int32_t)data->par_t2) >> 11;
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
	var3 = ((var3) * ((int32_t)data->par_t3 << 4)) >> 14;
	data->t_fine = var2 + var3;
	data->calc_temp = ((data->t_fine * 5) + 128) >> 8;
}

static void bme688_calc_press(struct bme688_data *data, uint32_t adc_press)
{
	int32_t var1, var2, var3, calc_press;

	var1 = (((int32_t)data->t_fine) >> 1) - 64000;
	var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) *
		(int32_t)data->par_p6) >> 2;
	var2 = var2 + ((var1 * (int32_t)data->par_p5) << 1);
	var2 = (var2 >> 2) + ((int32_t)data->par_p4 << 16);
	var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
		 ((int32_t)data->par_p3 << 5)) >> 3)
	       + (((int32_t)data->par_p2 * var1) >> 1);
	var1 = var1 >> 18;
	var1 = ((32768 + var1) * (int32_t)data->par_p1) >> 15;
	calc_press = 1048576 - adc_press;
	calc_press = (calc_press - (var2 >> 12)) * ((uint32_t)3125);
	/* This max value is used to provide precedence to multiplication or
	 * division in the pressure calculation equation to achieve least
	 * loss of precision and avoiding overflows.
	 * i.e Comparing value, signed int 32bit (1 << 30)
	 */
	if (calc_press >= (int32_t)0x40000000) {
		calc_press = ((calc_press / var1) << 1);
	} else {
		calc_press = ((calc_press << 1) / var1);
	}
	var1 = ((int32_t)data->par_p9 *
		(int32_t)(((calc_press >> 3)
			 * (calc_press >> 3)) >> 13)) >> 12;
	var2 = ((int32_t)(calc_press >> 2) * (int32_t)data->par_p8) >> 13;
	var3 = ((int32_t)(calc_press >> 8) * (int32_t)(calc_press >> 8)
		* (int32_t)(calc_press >> 8)
		* (int32_t)data->par_p10) >> 17;

	data->calc_press = calc_press
			   + ((var1 + var2 + var3
			       + ((int32_t)data->par_p7 << 7)) >> 4);
}

static void bme688_calc_humidity(struct bme688_data *data, uint16_t adc_humidity)
{
	int32_t var1, var2_1, var2_2, var2, var3, var4, var5, var6;
	int32_t temp_scaled, calc_hum;

	temp_scaled = (((int32_t)data->t_fine * 5) + 128) >> 8;
	var1 = (int32_t)(adc_humidity - ((int32_t)((int32_t)data->par_h1 * 16))) -
	       (((temp_scaled * (int32_t)data->par_h3)
		 / ((int32_t)100)) >> 1);
	var2_1 = (int32_t)data->par_h2;
	var2_2 = ((temp_scaled * (int32_t)data->par_h4) / (int32_t)100)
		 + (((temp_scaled * ((temp_scaled * (int32_t)data->par_h5)
				     / ((int32_t)100))) >> 6) / ((int32_t)100))
		 +  (int32_t)(1 << 14);
	var2 = (var2_1 * var2_2) >> 10;
	var3 = var1 * var2;
	var4 = (int32_t)data->par_h6 << 7;
	var4 = ((var4) + ((temp_scaled * (int32_t)data->par_h7) /
			  ((int32_t)100))) >> 4;
	var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
	var6 = (var4 * var5) >> 1;
	calc_hum = (((var3 + var6) >> 10) * ((int32_t)1000)) >> 12;

	if (calc_hum > 100000) { /* Cap at 100%rH */
		calc_hum = 100000;
	} else if (calc_hum < 0) {
		calc_hum = 0;
	}

	data->calc_humidity = calc_hum;
}

static void bme688_calc_gas_resistance(struct bme688_data *data, uint8_t gas_range,
				       uint16_t adc_gas_res)
{
	int64_t var1, var3;
	uint64_t var2;

	static const uint32_t look_up1[16] = { 2147483647, 2147483647, 2147483647,
			       2147483647, 2147483647, 2126008810, 2147483647,
			       2130303777, 2147483647, 2147483647, 2143188679,
			       2136746228, 2147483647, 2126008810, 2147483647,
			       2147483647 };

	static const uint32_t look_up2[16] = { 4096000000, 2048000000, 1024000000,
			       512000000, 255744255, 127110228, 64000000,
			       32258064, 16016016, 8000000, 4000000, 2000000,
			       1000000, 500000, 250000, 125000 };

	var1 = (int64_t)((1340 + (5 * (int64_t)data->range_sw_err)) *
		       ((int64_t)look_up1[gas_range])) >> 16;
	var2 = (((int64_t)((int64_t)adc_gas_res << 15) - (int64_t)(16777216)) + var1);
	var3 = (((int64_t)look_up2[gas_range] * (int64_t)var1) >> 9);
	data->calc_gas_resistance = (uint32_t)((var3 + ((int64_t)var2 >> 1))
					    / (int64_t)var2);
}

static uint8_t bme688_calc_res_heat(struct bme688_data *data, uint16_t heatr_temp)
{
	uint8_t heatr_res;
	int32_t var1, var2, var3, var4, var5;
	int32_t heatr_res_x100;
	int32_t amb_temp = 25;    /* Assume ambient temperature to be 25 deg C */

	if (heatr_temp > 400) { /* Cap temperature */
		heatr_temp = 400;
	}

	var1 = ((amb_temp * data->par_gh3) / 1000) * 256;
	var2 = (data->par_gh1 + 784) * (((((data->par_gh2 + 154009)
					   * heatr_temp * 5) / 100)
					 + 3276800) / 10);
	var3 = var1 + (var2 / 2);
	var4 = (var3 / (data->res_heat_range + 4));
	var5 = (131 * data->res_heat_val) + 65536;
	heatr_res_x100 = ((var4 / var5) - 250) * 34;
	heatr_res = (heatr_res_x100 + 50) / 100;

	return heatr_res;
}

static uint8_t bme688_calc_gas_wait(uint16_t dur)
{
	uint8_t factor = 0, durval;

	if (dur >= 0xfc0) {
		durval = 0xff; /* Max duration*/
	} else {
		while (dur > 0x3F) {
			dur = dur / 4;
			factor += 1;
		}
		durval = dur + (factor * 64);
	}

	return durval;
}

static int bme688_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct bme688_data *data = dev->data;
	uint8_t buff[BME688_LEN_FIELD] = { 0 };
	uint8_t gas_range;
	uint32_t adc_temp, adc_press;
	uint16_t adc_hum, adc_gas_res;
	int size = BME688_LEN_FIELD;
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	ret = bme688_reg_read(dev, BME688_REG_FIELD0, buff, size);
	if (ret < 0) {
		return ret;
	}

	data->new_data = buff[0] & BME688_MSK_NEW_DATA;
	data->heatr_stab = buff[14] & BME688_MSK_HEATR_STAB;

	adc_press = (uint32_t)(((uint32_t)buff[2] << 12) | ((uint32_t)buff[3] << 4)
			    | ((uint32_t)buff[4] >> 4));
	adc_temp = (uint32_t)(((uint32_t)buff[5] << 12) | ((uint32_t)buff[6] << 4)
			   | ((uint32_t)buff[7] >> 4));
	adc_hum = (uint16_t)(((uint32_t)buff[8] << 8) | (uint32_t)buff[9]);
	adc_gas_res = (uint16_t)((uint32_t)buff[13] << 2 | (((uint32_t)buff[14]) >> 6));
	gas_range = buff[14] & BME688_MSK_GAS_RANGE;

	if (data->new_data) {
		bme688_calc_temp(data, adc_temp);
		bme688_calc_press(data, adc_press);
		bme688_calc_humidity(data, adc_hum);
		bme688_calc_gas_resistance(data, gas_range, adc_gas_res);
	}

	/* Trigger the next measurement */
	ret = bme688_reg_write(dev, BME688_REG_CTRL_MEAS,
			       BME688_CTRL_MEAS_VAL);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int bme688_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bme688_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		/*
		 * data->calc_temp has a resolution of 0.01 degC.
		 * So 5123 equals 51.23 degC.
		 */
		val->val1 = data->calc_temp / 100;
		val->val2 = data->calc_temp % 100 * 10000;
		break;
	case SENSOR_CHAN_PRESS:
		/*
		 * data->calc_press has a resolution of 1 Pa.
		 * So 96321 equals 96.321 kPa.
		 */
		val->val1 = data->calc_press / 1000;
		val->val2 = (data->calc_press % 1000) * 1000;
		break;
	case SENSOR_CHAN_HUMIDITY:
		/*
		 * data->calc_humidity has a resolution of 0.001 %RH.
		 * So 46333 equals 46.333 %RH.
		 */
		val->val1 = data->calc_humidity / 1000;
		val->val2 = (data->calc_humidity % 1000) * 1000;
		break;
	case SENSOR_CHAN_GAS_RES:
		/*
		 * data->calc_gas_resistance has a resolution of 1 ohm.
		 * So 100000 equals 100000 ohms.
		 */
		val->val1 = data->calc_gas_resistance;
		val->val2 = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bme688_read_compensation(const struct device *dev)
{
	struct bme688_data *data = dev->data;
	uint8_t buff[BME688_LEN_COEFF_ALL];
	int err = 0;

	err = bme688_reg_read(dev, BME688_REG_COEFF1, buff, BME688_LEN_COEFF1);
	if (err < 0) {
		return err;
	}

	err = bme688_reg_read(dev, BME688_REG_COEFF2, &buff[BME688_LEN_COEFF1],
			      16);
	if (err < 0) {
		return err;
	}

	err = bme688_reg_read(dev, BME688_REG_COEFF3,
			      &buff[BME688_LEN_COEFF1 + BME688_LEN_COEFF2],
			      BME688_LEN_COEFF3);
	if (err < 0) {
		return err;
	}

	/* Temperature related coefficients */
	data->par_t1 = (uint16_t)(BME688_CONCAT_BYTES(buff[32], buff[31]));
	data->par_t2 = (int16_t)(BME688_CONCAT_BYTES(buff[1], buff[0]));
	data->par_t3 = (uint8_t)(buff[2]);

	/* Pressure related coefficients */
	data->par_p1 = (uint16_t)(BME688_CONCAT_BYTES(buff[5], buff[4]));
	data->par_p2 = (int16_t)(BME688_CONCAT_BYTES(buff[7], buff[6]));
	data->par_p3 = (int8_t)buff[8];
	data->par_p4 = (int16_t)(BME688_CONCAT_BYTES(buff[11], buff[10]));
	data->par_p5 = (int16_t)(BME688_CONCAT_BYTES(buff[13], buff[12]));
	data->par_p6 = (int8_t)(buff[15]);
	data->par_p7 = (int8_t)(buff[14]);
	data->par_p8 = (int16_t)(BME688_CONCAT_BYTES(buff[19], buff[18]));
	data->par_p9 = (int16_t)(BME688_CONCAT_BYTES(buff[21], buff[20]));
	data->par_p10 = (uint8_t)(buff[22]);

	/* Humidity related coefficients */
	data->par_h1 = (uint16_t)(((uint16_t)buff[25] << 4) | (buff[24] & 0x0f));
	data->par_h2 = (uint16_t)(((uint16_t)buff[23] << 4) | ((buff[24]) >> 4));
	data->par_h3 = (int8_t)buff[26];
	data->par_h4 = (int8_t)buff[27];
	data->par_h5 = (int8_t)buff[28];
	data->par_h6 = (uint8_t)buff[29];
	data->par_h7 = (int8_t)buff[30];

	/* Gas heater related coefficients */
	data->par_gh1 = (int8_t)buff[35];
	data->par_gh2 = (int16_t)(BME688_CONCAT_BYTES(buff[34], buff[33]));
	data->par_gh3 = (int8_t)buff[36];

	data->res_heat_val = (int8_t)buff[37];
	data->res_heat_range = ((buff[39] & BME688_MSK_RH_RANGE) >> 4);
	data->range_sw_err = ((int8_t)(buff[41] & BME688_MSK_RANGE_SW_ERR)) / 16;

	return 0;
}

static int bme688_chip_init(const struct device *dev)
{
	struct bme688_data *data = dev->data;
	int err;

	err = bme688_reg_read(dev, BME688_REG_CHIP_ID, &data->chip_id, 1);
	if (err < 0) {
		return err;
	}

	if (data->chip_id == BME688_CHIP_ID) {
		LOG_DBG("BME688 chip detected");
	} else {
		LOG_ERR("Bad BME688 chip id 0x%x", data->chip_id);
		return -ENOTSUP;
	}

	err = bme688_read_compensation(dev);
	if (err < 0) {
		return err;
	}

	err = bme688_reg_write(dev, BME688_REG_CTRL_HUM, BME688_HUMIDITY_OVER);
	if (err < 0) {
		return err;
	}

	err = bme688_reg_write(dev, BME688_REG_CONFIG, BME688_CONFIG_VAL);
	if (err < 0) {
		return err;
	}

	err = bme688_reg_write(dev, BME688_REG_CTRL_GAS_1,
			       BME688_CTRL_GAS_1_VAL);
	if (err < 0) {
		return err;
	}

	err = bme688_reg_write(dev, BME688_REG_RES_HEAT0,
			       bme688_calc_res_heat(data, BME688_HEATR_TEMP));
	if (err < 0) {
		return err;
	}

	err = bme688_reg_write(dev, BME688_REG_GAS_WAIT0,
			       bme688_calc_gas_wait(BME688_HEATR_DUR_MS));
	if (err < 0) {
		return err;
	}

	err = bme688_reg_write(dev, BME688_REG_CTRL_MEAS,
			       BME688_CTRL_MEAS_VAL);
	if (err < 0) {
		return err;
	}

	return 0;
}

static int bme688_init(const struct device *dev)
{
	const struct bme688_config *config = dev->config;

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C master %s not ready", config->bus.bus->name);
		return -EINVAL;
	}

	if (bme688_chip_init(dev) < 0) {
		return -EINVAL;
	}

	return 0;
}

static const struct sensor_driver_api bme688_api_funcs = {
	.sample_fetch = bme688_sample_fetch,
	.channel_get = bme688_channel_get,
};

static struct bme688_data bme688_data;

static const struct bme688_config bme688_config = {
	.bus = I2C_DT_SPEC_INST_GET(0)
};

DEVICE_DT_INST_DEFINE(0, bme688_init, NULL, &bme688_data,
		      &bme688_config, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		      &bme688_api_funcs);
