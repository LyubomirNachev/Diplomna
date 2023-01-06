/*
 * Copyright (c) 2016, 2017 Intel Corporation
 * Copyright (c) 2017 IpTronix S.r.l.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BMP280_BMP280_H_
#define ZEPHYR_DRIVERS_SENSOR_BMP280_BMP280_H_

#include <zephyr/types.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/spi.h>
#include <drivers/i2c.h>

#define DT_DRV_COMPAT bosch_bmp280

#define BMP280_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#define BMP280_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

union bmp280_bus {
#if BMP280_BUS_SPI
	struct spi_dt_spec spi;
#endif
#if BMP280_BUS_I2C
	struct i2c_dt_spec i2c;
#endif
};

typedef int (*bmp280_bus_check_fn)(const union bmp280_bus *bus);
typedef int (*bmp280_reg_read_fn)(const union bmp280_bus *bus,
				  uint8_t start, uint8_t *buf, int size);
typedef int (*bmp280_reg_write_fn)(const union bmp280_bus *bus,
				   uint8_t reg, uint8_t val);

struct bmp280_bus_io {
	bmp280_bus_check_fn check;
	bmp280_reg_read_fn read;
	bmp280_reg_write_fn write;
};

#if BMP280_BUS_SPI
#define BMP280_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB |	\
			      SPI_MODE_CPOL | SPI_MODE_CPHA)
extern const struct bmp280_bus_io bmp280_bus_io_spi;
#endif

#if BMP280_BUS_I2C
extern const struct bmp280_bus_io bmp280_bus_io_i2c;
#endif

#define BMP280_REG_PRESS_MSB            0xF7
#define BMP280_REG_COMP_START           0x88
#define BMP280_REG_HUM_COMP_PART1       0xA1
#define BMP280_REG_HUM_COMP_PART2       0xE1
#define BMP280_REG_ID                   0xD0
#define BMP280_REG_CONFIG               0xF5
#define BMP280_REG_CTRL_MEAS            0xF4
#define BMP280_REG_CTRL_HUM             0xF2
#define BMP280_REG_STATUS               0xF3
#define BMP280_REG_RESET                0xE0

#define BMP280_CHIP_ID_SAMPLE_1         0x56
#define BMP280_CHIP_ID_SAMPLE_2         0x57
#define BMP280_CHIP_ID_MP               0x58
#define BMP280_CHIP_ID                  0x60
#define BMP280_MODE_SLEEP               0x00
#define BMP280_MODE_FORCED              0x01
#define BMP280_MODE_NORMAL              0x03
#define BMP280_SPI_3W_DISABLE           0x00
#define BMP280_CMD_SOFT_RESET           0xB6
#define BMP280_STATUS_MEASURING         0x08
#define BMP280_STATUS_IM_UPDATE         0x01

#if defined CONFIG_BMP280_MODE_NORMAL
#define BMP280_MODE BMP280_MODE_NORMAL
#elif defined CONFIG_BMP280_MODE_FORCED
#define BMP280_MODE BMP280_MODE_FORCED
#endif

#if defined CONFIG_BMP280_TEMP_OVER_1X
#define BMP280_TEMP_OVER                (1 << 5)
#elif defined CONFIG_BMP280_TEMP_OVER_2X
#define BMP280_TEMP_OVER                (2 << 5)
#elif defined CONFIG_BMP280_TEMP_OVER_4X
#define BMP280_TEMP_OVER                (3 << 5)
#elif defined CONFIG_BMP280_TEMP_OVER_8X
#define BMP280_TEMP_OVER                (4 << 5)
#elif defined CONFIG_BMP280_TEMP_OVER_16X
#define BMP280_TEMP_OVER                (5 << 5)
#endif

#if defined CONFIG_BMP280_PRESS_OVER_1X
#define BMP280_PRESS_OVER               (1 << 2)
#elif defined CONFIG_BMP280_PRESS_OVER_2X
#define BMP280_PRESS_OVER               (2 << 2)
#elif defined CONFIG_BMP280_PRESS_OVER_4X
#define BMP280_PRESS_OVER               (3 << 2)
#elif defined CONFIG_BMP280_PRESS_OVER_8X
#define BMP280_PRESS_OVER               (4 << 2)
#elif defined CONFIG_BMP280_PRESS_OVER_16X
#define BMP280_PRESS_OVER               (5 << 2)
#endif

#if defined CONFIG_BMP280_HUMIDITY_OVER_1X
#define BMP280_HUMIDITY_OVER            1
#elif defined CONFIG_BMP280_HUMIDITY_OVER_2X
#define BMP280_HUMIDITY_OVER            2
#elif defined CONFIG_BMP280_HUMIDITY_OVER_4X
#define BMP280_HUMIDITY_OVER            3
#elif defined CONFIG_BMP280_HUMIDITY_OVER_8X
#define BMP280_HUMIDITY_OVER            4
#elif defined CONFIG_BMP280_HUMIDITY_OVER_16X
#define BMP280_HUMIDITY_OVER            5
#endif

#if defined CONFIG_BMP280_STANDBY_05MS
#define BMP280_STANDBY                  0
#elif defined CONFIG_BMP280_STANDBY_62MS
#define BMP280_STANDBY                  (1 << 5)
#elif defined CONFIG_BMP280_STANDBY_125MS
#define BMP280_STANDBY                  (2 << 5)
#elif defined CONFIG_BMP280_STANDBY_250MS
#define BMP280_STANDBY                  (3 << 5)
#elif defined CONFIG_BMP280_STANDBY_500MS
#define BMP280_STANDBY                  (4 << 5)
#elif defined CONFIG_BMP280_STANDBY_1000MS
#define BMP280_STANDBY                  (5 << 5)
#elif defined CONFIG_BMP280_STANDBY_2000MS
#define BMP280_STANDBY                  (6 << 5)
#elif defined CONFIG_BMP280_STANDBY_4000MS
#define BMP280_STANDBY                  (7 << 5)
#endif

#if defined CONFIG_BMP280_FILTER_OFF
#define BMP280_FILTER                   0
#elif defined CONFIG_BMP280_FILTER_2
#define BMP280_FILTER                   (1 << 2)
#elif defined CONFIG_BMP280_FILTER_4
#define BMP280_FILTER                   (2 << 2)
#elif defined CONFIG_BMP280_FILTER_8
#define BMP280_FILTER                   (3 << 2)
#elif defined CONFIG_BMP280_FILTER_16
#define BMP280_FILTER                   (4 << 2)
#endif

#define BMP280_CTRL_MEAS_VAL            (BMP280_PRESS_OVER | \
					 BMP280_TEMP_OVER |  \
					 BMP280_MODE)
#define BMP280_CONFIG_VAL               (BMP280_STANDBY | \
					 BMP280_FILTER |  \
					 BMP280_SPI_3W_DISABLE)


#define BMP280_CTRL_MEAS_OFF_VAL	(BMP280_PRESS_OVER | \
					 BMP280_TEMP_OVER |  \
					 BMP280_MODE_SLEEP)

#endif /* ZEPHYR_DRIVERS_SENSOR_BMP280_BMP280_H_ */
