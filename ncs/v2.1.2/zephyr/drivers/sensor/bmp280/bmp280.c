/* bmp280.c - Driver for Bosch BMP280 temperature and pressure sensor */

/*
 * Copyright (c) 2016, 2017 Intel Corporation
 * Copyright (c) 2017 IpTronix S.r.l.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <pm/device.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>

#include <logging/log.h>

#include "bmp280.h"

LOG_MODULE_REGISTER(BMP280, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "BMP280 driver enabled without any devices"
#endif

struct bmp280_data {
	/* Compensation parameters. */
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
	uint16_t dig_p1;
	int16_t dig_p2;
	int16_t dig_p3;
	int16_t dig_p4;
	int16_t dig_p5;
	int16_t dig_p6;
	int16_t dig_p7;
	int16_t dig_p8;
	int16_t dig_p9;
	uint8_t dig_h1;
	int16_t dig_h2;
	uint8_t dig_h3;
	int16_t dig_h4;
	int16_t dig_h5;
	int8_t dig_h6;

	/* Compensated values. */
	int32_t comp_temp;
	uint32_t comp_press;
	uint32_t comp_humidity;

	/* Carryover between temperature and pressure/humidity compensation. */
	int32_t t_fine;

	uint8_t chip_id;
};

struct bmp280_config {
	union bmp280_bus bus;
	const struct bmp280_bus_io *bus_io;
};

static inline int bmp280_bus_check(const struct device *dev)
{
	const struct bmp280_config *cfg = dev->config;

	return cfg->bus_io->check(&cfg->bus);
}

static inline int bmp280_reg_read(const struct device *dev,
				  uint8_t start, uint8_t *buf, int size)
{
	const struct bmp280_config *cfg = dev->config;

	return cfg->bus_io->read(&cfg->bus, start, buf, size);
}

static inline int bmp280_reg_write(const struct device *dev, uint8_t reg,
				   uint8_t val)
{
	const struct bmp280_config *cfg = dev->config;

	return cfg->bus_io->write(&cfg->bus, reg, val);
}

/*
 * Compensation code taken from BMP280 datasheet, Section 4.2.3
 * "Compensation formula".
 */
static void bmp280_compensate_temp(struct bmp280_data *data, int32_t adc_temp)
{
	int32_t var1, var2;

	var1 = (((adc_temp >> 3) - ((int32_t)data->dig_t1 << 1)) *
		((int32_t)data->dig_t2)) >> 11;
	var2 = (((((adc_temp >> 4) - ((int32_t)data->dig_t1)) *
		  ((adc_temp >> 4) - ((int32_t)data->dig_t1))) >> 12) *
		((int32_t)data->dig_t3)) >> 14;

	data->t_fine = var1 + var2;
	data->comp_temp = (data->t_fine * 5 + 128) >> 8;
}

static void bmp280_compensate_press(struct bmp280_data *data, int32_t adc_press)
{
	int64_t var1, var2, p;

	var1 = ((int64_t)data->t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)data->dig_p6;
	var2 = var2 + ((var1 * (int64_t)data->dig_p5) << 17);
	var2 = var2 + (((int64_t)data->dig_p4) << 35);
	var1 = ((var1 * var1 * (int64_t)data->dig_p3) >> 8) +
		((var1 * (int64_t)data->dig_p2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)data->dig_p1) >> 33;

	/* Avoid exception caused by division by zero. */
	if (var1 == 0) {
		data->comp_press = 0U;
		return;
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)data->dig_p9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)data->dig_p8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)data->dig_p7) << 4);

	data->comp_press = (uint32_t)p;
}

static void bmp280_compensate_humidity(struct bmp280_data *data,
				       int32_t adc_humidity)
{
	int32_t h;

	h = (data->t_fine - ((int32_t)76800));
	h = ((((adc_humidity << 14) - (((int32_t)data->dig_h4) << 20) -
		(((int32_t)data->dig_h5) * h)) + ((int32_t)16384)) >> 15) *
		(((((((h * ((int32_t)data->dig_h6)) >> 10) * (((h *
		((int32_t)data->dig_h3)) >> 11) + ((int32_t)32768))) >> 10) +
		((int32_t)2097152)) * ((int32_t)data->dig_h2) + 8192) >> 14);
	h = (h - (((((h >> 15) * (h >> 15)) >> 7) *
		((int32_t)data->dig_h1)) >> 4));
	h = (h > 419430400 ? 419430400 : h);

	data->comp_humidity = (uint32_t)(h >> 12);
}

static int bmp280_wait_until_ready(const struct device *dev)
{
	uint8_t status = 0;
	int ret;

	/* Wait for NVM to copy and and measurement to be completed */
	do {
		k_sleep(K_MSEC(3));
		ret = bmp280_reg_read(dev, BMP280_REG_STATUS, &status, 1);
		if (ret < 0) {
			return ret;
		}
	} while (status & (BMP280_STATUS_MEASURING | BMP280_STATUS_IM_UPDATE));

	return 0;
}

static int bmp280_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct bmp280_data *data = dev->data;
	uint8_t buf[8];
	int32_t adc_press, adc_temp, adc_humidity;
	int size = 6;
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

#ifdef CONFIG_PM_DEVICE
	enum pm_device_state state;
	(void)pm_device_state_get(dev, &state);
	/* Do not allow sample fetching from suspended state */
	if (state == PM_DEVICE_STATE_SUSPENDED)
		return -EIO;
#endif

#ifdef CONFIG_BMP280_MODE_FORCED
	ret = bmp280_reg_write(dev, BMP280_REG_CTRL_MEAS, BMP280_CTRL_MEAS_VAL);
	if (ret < 0) {
		return ret;
	}
#endif

	ret = bmp280_wait_until_ready(dev);
	if (ret < 0) {
		return ret;
	}

	if (data->chip_id == BMP280_CHIP_ID) {
		size = 8;
	}
	ret = bmp280_reg_read(dev, BMP280_REG_PRESS_MSB, buf, size);
	if (ret < 0) {
		return ret;
	}

	adc_press = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
	adc_temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);

	bmp280_compensate_temp(data, adc_temp);
	bmp280_compensate_press(data, adc_press);

	if (data->chip_id == BMP280_CHIP_ID) {
		adc_humidity = (buf[6] << 8) | buf[7];
		bmp280_compensate_humidity(data, adc_humidity);
	}

	return 0;
}

static int bmp280_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bmp280_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		/*
		 * data->comp_temp has a resolution of 0.01 degC.  So
		 * 5123 equals 51.23 degC.
		 */
		val->val1 = data->comp_temp / 100;
		val->val2 = data->comp_temp % 100 * 10000;
		break;
	case SENSOR_CHAN_PRESS:
		/*
		 * data->comp_press has 24 integer bits and 8
		 * fractional.  Output value of 24674867 represents
		 * 24674867/256 = 96386.2 Pa = 963.862 hPa
		 */
		val->val1 = (data->comp_press >> 8) / 1000U;
		val->val2 = (data->comp_press >> 8) % 1000 * 1000U +
			(((data->comp_press & 0xff) * 1000U) >> 8);
		break;
	case SENSOR_CHAN_HUMIDITY:
		/*
		 * data->comp_humidity has 22 integer bits and 10
		 * fractional.  Output value of 47445 represents
		 * 47445/1024 = 46.333 %RH
		 */
		val->val1 = (data->comp_humidity >> 10);
		val->val2 = (((data->comp_humidity & 0x3ff) * 1000U * 1000U) >> 10);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct sensor_driver_api bmp280_api_funcs = {
	.sample_fetch = bmp280_sample_fetch,
	.channel_get = bmp280_channel_get,
};

static int bmp280_read_compensation(const struct device *dev)
{
	struct bmp280_data *data = dev->data;
	uint16_t buf[12];
	uint8_t hbuf[7];
	int err = 0;

	err = bmp280_reg_read(dev, BMP280_REG_COMP_START,
			      (uint8_t *)buf, sizeof(buf));

	if (err < 0) {
		LOG_DBG("COMP_START read failed: %d", err);
		return err;
	}

	data->dig_t1 = sys_le16_to_cpu(buf[0]);
	data->dig_t2 = sys_le16_to_cpu(buf[1]);
	data->dig_t3 = sys_le16_to_cpu(buf[2]);

	data->dig_p1 = sys_le16_to_cpu(buf[3]);
	data->dig_p2 = sys_le16_to_cpu(buf[4]);
	data->dig_p3 = sys_le16_to_cpu(buf[5]);
	data->dig_p4 = sys_le16_to_cpu(buf[6]);
	data->dig_p5 = sys_le16_to_cpu(buf[7]);
	data->dig_p6 = sys_le16_to_cpu(buf[8]);
	data->dig_p7 = sys_le16_to_cpu(buf[9]);
	data->dig_p8 = sys_le16_to_cpu(buf[10]);
	data->dig_p9 = sys_le16_to_cpu(buf[11]);

	if (data->chip_id == BMP280_CHIP_ID) {
		err = bmp280_reg_read(dev, BMP280_REG_HUM_COMP_PART1,
				      &data->dig_h1, 1);
		if (err < 0) {
			LOG_DBG("HUM_COMP_PART1 read failed: %d", err);
			return err;
		}

		err = bmp280_reg_read(dev, BMP280_REG_HUM_COMP_PART2, hbuf, 7);
		if (err < 0) {
			LOG_DBG("HUM_COMP_PART2 read failed: %d", err);
			return err;
		}

		data->dig_h2 = (hbuf[1] << 8) | hbuf[0];
		data->dig_h3 = hbuf[2];
		data->dig_h4 = (hbuf[3] << 4) | (hbuf[4] & 0x0F);
		data->dig_h5 = ((hbuf[4] >> 4) & 0x0F) | (hbuf[5] << 4);
		data->dig_h6 = hbuf[6];
	}

	return 0;
}

static int bmp280_chip_init(const struct device *dev)
{
	struct bmp280_data *data = dev->data;
	int err;

	err = bmp280_bus_check(dev);
	if (err < 0) {
		LOG_DBG("bus check failed: %d", err);
		return err;
	}

	err = bmp280_reg_read(dev, BMP280_REG_ID, &data->chip_id, 1);
	if (err < 0) {
		LOG_DBG("ID read failed: %d", err);
		return err;
	}

	if (data->chip_id == BMP280_CHIP_ID) {
		LOG_DBG("ID OK");
	} else if (data->chip_id == BMP280_CHIP_ID_MP ||
		   data->chip_id == BMP280_CHIP_ID_SAMPLE_1) {
		LOG_DBG("ID OK (BMP280)");
	} else {
		LOG_DBG("bad chip id 0x%x", data->chip_id);
		return -ENOTSUP;
	}

	err = bmp280_reg_write(dev, BMP280_REG_RESET, BMP280_CMD_SOFT_RESET);
	if (err < 0) {
		LOG_DBG("Soft-reset failed: %d", err);
	}

	err = bmp280_wait_until_ready(dev);
	if (err < 0) {
		return err;
	}

	err = bmp280_read_compensation(dev);
	if (err < 0) {
		return err;
	}

	if (data->chip_id == BMP280_CHIP_ID) {
		err = bmp280_reg_write(dev, BMP280_REG_CTRL_HUM,
				       BMP280_HUMIDITY_OVER);
		if (err < 0) {
			LOG_DBG("CTRL_HUM write failed: %d", err);
			return err;
		}
	}

	err = bmp280_reg_write(dev, BMP280_REG_CTRL_MEAS,
			       BMP280_CTRL_MEAS_VAL);
	if (err < 0) {
		LOG_DBG("CTRL_MEAS write failed: %d", err);
		return err;
	}

	err = bmp280_reg_write(dev, BMP280_REG_CONFIG,
			       BMP280_CONFIG_VAL);
	if (err < 0) {
		LOG_DBG("CONFIG write failed: %d", err);
		return err;
	}
	/* Wait for the sensor to be ready */
	k_sleep(K_MSEC(1));

	LOG_DBG("\"%s\" OK", dev->name);
	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int bmp280_pm_action(const struct device *dev,
			    enum pm_device_action action)
{
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		/* Re-initialize the chip */
		ret = bmp280_chip_init(dev);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		/* Put the chip into sleep mode */
		ret = bmp280_reg_write(dev,
			BMP280_REG_CTRL_MEAS,
			BMP280_CTRL_MEAS_OFF_VAL);

		if (ret < 0) {
			LOG_DBG("CTRL_MEAS write failed: %d", ret);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

/* Initializes a struct bmp280_config for an instance on a SPI bus. */
#define BMP280_CONFIG_SPI(inst)				\
	{						\
		.bus.spi = SPI_DT_SPEC_INST_GET(	\
			inst, BMP280_SPI_OPERATION, 0),	\
		.bus_io = &bmp280_bus_io_spi,		\
	}

/* Initializes a struct bmp280_config for an instance on an I2C bus. */
#define BMP280_CONFIG_I2C(inst)			       \
	{					       \
		.bus.i2c = I2C_DT_SPEC_INST_GET(inst), \
		.bus_io = &bmp280_bus_io_i2c,	       \
	}

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define BMP280_DEFINE(inst)						\
	static struct bmp280_data bmp280_data_##inst;			\
	static const struct bmp280_config bmp280_config_##inst =	\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),			\
			    (BMP280_CONFIG_SPI(inst)),			\
			    (BMP280_CONFIG_I2C(inst)));			\
									\
	PM_DEVICE_DT_INST_DEFINE(inst, bmp280_pm_action);		\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			 bmp280_chip_init,				\
			 PM_DEVICE_DT_INST_GET(inst),			\
			 &bmp280_data_##inst,				\
			 &bmp280_config_##inst,				\
			 POST_KERNEL,					\
			 CONFIG_SENSOR_INIT_PRIORITY,			\
			 &bmp280_api_funcs);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(BMP280_DEFINE)
