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

#define BH1750_CHM 0b00010000
#define BH1750_CHM_2 0b00010001
#define BH1750_CLM 0b00010011
#define BH1750_OTH 0b00100000
#define BH1750_OTH_2 0b00100001
#define BH1750_OTL 0b00100011 



enum BH1750Mode {
  CHM = 0b00010000,     //CHM: Continuously H-Resolution Mode
  CHM_2 = 0b00010001,   //CHM_2: Continuously H-Resolution Mode2
  CLM = 0b00010011,     //CLM: Continuously L-Resolution Mode
  OTH = 0b00100000,     //OTH: One Time H-Resolution Mode
  OTH_2 = 0b00100001,   //OTH_2: One Time H-Resolution Mode2
  OTL = 0b00100011      //OTL: One Time L-Resolution Mode
} mode;






// #define ISL29035_COMMAND_I_REG		0x00
// #define ISL29035_OPMODE_SHIFT		5
// #define ISL29035_OPMODE_MASK		(7 << ISL29035_OPMODE_SHIFT)
// #define ISL29035_INT_BIT_SHIFT		2
// #define ISL29035_INT_BIT_MASK		(1 << ISL29035_INT_BIT_SHIFT)
// #define ISL29035_INT_PRST_SHIFT		0
// #define ISL29035_INT_PRST_MASK		(3 << ISL29035_INT_BIT_SHIFT)

// #define ISL29035_OPMODE_OFF		0
// #define ISL29035_OPMODE_ALS_ONCE	1
// #define ISL29035_OPMODE_IR_ONCE		2
// #define ISL29035_OPMODE_ALS_CONT	5
// #define ISL29035_OPMODE_IR_CONT		6

// #define ISL29035_COMMAND_II_REG		0x01
// #define ISL29035_LUX_RANGE_SHIFT	0
// #define ISL29035_LUX_RANGE_MASK		(3 << ISL29035_LUX_RANGE_SHIFT)
// #define ISL29035_ADC_RES_SHIFT		2
// #define ISL29035_ADC_RES_MASK		(3 << ISL29035_ADC_RES_SHIFT)

// #define ISL29035_DATA_LSB_REG		0x02
// #define ISL29035_DATA_MSB_REG		0x03
// #define ISL29035_INT_LT_LSB_REG		0x04
// #define ISL29035_INT_LT_MSB_REG		0x05
// #define ISL29035_INT_HT_LSB_REG		0x06
// #define ISL29035_INT_HT_MSB_REG		0x07

// #define ISL29035_ID_REG			0x0F
// #define ISL29035_BOUT_SHIFT		7
// #define ISL29035_BOUT_MASK		(1 << ISL29035_BOUT_SHIFT)
// #define ISL29035_ID_SHIFT		3
// #define ISL29035_ID_MASK		(3 << ISL29035_ID_SHIFT)

// #if CONFIG_ISL29035_MODE_ALS
// 	#define ISL29035_ACTIVE_OPMODE		ISL29035_OPMODE_ALS_CONT
// 	#define ISL29035_ACTIVE_CHAN		SENSOR_CHAN_LIGHT
// #elif CONFIG_ISL29035_MODE_IR
// 	#define ISL29035_ACTIVE_OPMODE		ISL29035_OPMODE_IR_CONT
// 	#define ISL29035_ACTIVE_CHAN		SENSOR_CHAN_IR
// #endif

// #define ISL29035_ACTIVE_OPMODE_BITS		\
// 	(ISL29035_ACTIVE_OPMODE << ISL29035_OPMODE_SHIFT)

// #if CONFIG_ISL29035_LUX_RANGE_1K
// 	#define ISL29035_LUX_RANGE_IDX		0
// 	#define ISL29035_LUX_RANGE		1000
// #elif CONFIG_ISL29035_LUX_RANGE_4K
// 	#define ISL29035_LUX_RANGE_IDX		1
// 	#define ISL29035_LUX_RANGE		4000
// #elif CONFIG_ISL29035_LUX_RANGE_16K
// 	#define ISL29035_LUX_RANGE_IDX		2
// 	#define ISL29035_LUX_RANGE		16000
// #elif CONFIG_ISL29035_LUX_RANGE_64K
// 	#define ISL29035_LUX_RANGE_IDX		3
// 	#define ISL29035_LUX_RANGE		64000
// #endif

// #define ISL29035_LUX_RANGE_BITS			\
// 	(ISL29035_LUX_RANGE_IDX << ISL29035_LUX_RANGE_SHIFT)

// #if CONFIG_ISL29035_INTEGRATION_TIME_26
// 	#define ISL29035_ADC_RES_IDX		3
// #elif CONFIG_ISL29035_INTEGRATION_TIME_410
// 	#define ISL29035_ADC_RES_IDX		2
// #elif CONFIG_ISL29035_INTEGRATION_TIME_6500
// 	#define ISL29035_ADC_RES_IDX		1
// #elif CONFIG_ISL29035_INTEGRATION_TIME_105K
// 	#define ISL29035_ADC_RES_IDX		0
// #endif

// #define ISL29035_ADC_RES_BITS			\
// 	(ISL29035_ADC_RES_IDX << ISL29035_ADC_RES_SHIFT)

// #define ISL29035_ADC_DATA_BITS	(16 - 4 * ISL29035_ADC_RES_IDX)
// #define ISL29035_ADC_DATA_MASK	(0xFFFF >> (16 - ISL29035_ADC_DATA_BITS))

// #if CONFIG_ISL29035_INT_PERSIST_1
// 	#define ISL29035_INT_PRST_IDX		0
// 	#define ISL29035_INT_PRST_CYCLES	1
// #elif CONFIG_ISL29035_INT_PERSIST_4
// 	#define ISL29035_INT_PRST_IDX		1
// 	#define ISL29035_INT_PRST_CYCLES	4
// #elif CONFIG_ISL29035_INT_PERSIST_8
// 	#define ISL29035_INT_PRST_IDX		2
// 	#define ISL29035_INT_PRST_CYCLES	8
// #elif CONFIG_ISL29035_INT_PERSIST_16
// 	#define ISL29035_INT_PRST_IDX		3
// 	#define ISL29035_INT_PRST_CYCLES	16
// #endif

// #define ISL29035_INT_PRST_BITS			\
// 	(ISL29035_INT_PRST_IDX << ISL29035_INT_PRST_SHIFT)

// struct isl29035_driver_data {
// 	uint16_t data_sample;

// #if CONFIG_ISL29035_TRIGGER
// 	const struct device *dev;
// 	struct gpio_callback gpio_cb;

// 	struct sensor_trigger th_trigger;
// 	sensor_trigger_handler_t th_handler;

// #if defined(CONFIG_ISL29035_TRIGGER_OWN_THREAD)
// 	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ISL29035_THREAD_STACK_SIZE);
// 	struct k_thread thread;
// 	struct k_sem gpio_sem;
// #elif defined(CONFIG_ISL29035_TRIGGER_GLOBAL_THREAD)
// 	struct k_work work;
// #endif

// #endif /* CONFIG_ISL29035_TRIGGER */
// };

// struct isl29035_config {
// 	struct i2c_dt_spec i2c;
// #if CONFIG_ISL29035_TRIGGER
// 	struct gpio_dt_spec int_gpio;
// #endif
// };

// #ifdef CONFIG_ISL29035_TRIGGER
// int isl29035_attr_set(const struct device *dev,
// 		      enum sensor_channel chan,
// 		      enum sensor_attribute attr,
// 		      const struct sensor_value *val);

// int isl29035_trigger_set(const struct device *dev,
// 			 const struct sensor_trigger *trig,
// 			 sensor_trigger_handler_t handler);

// int isl29035_init_interrupt(const struct device *dev);
// #endif

#endif /* ZEPHYR_DRIVERS_SENSOR_BH1750_BH1750_H_ */
