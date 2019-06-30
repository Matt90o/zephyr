/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MLX90614_MLX90614_H_
#define ZEPHYR_DRIVERS_SENSOR_MLX90614_MLX90614_H_

#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>

#define MLX90614_I2C_ADDRESS		CONFIG_MLX90614_I2C_ADDR

#define MLX90614_REG_CONFIG		0x02
#define MLX90614_ALERT_EN_BIT		BIT(8)

#define MLX90614_REG_TOBJ			0x03
#define MLX90614_DATA_INVALID_BIT		BIT(0)

#define MLX90614_REG_STATUS		0x04
#define MLX90614_DATA_READY_INT_BIT	BIT(14)
#define MLX90614_TOBJ_TH_HIGH_INT_BIT	BIT(13)
#define MLX90614_TOBJ_TH_LOW_INT_BIT	BIT(12)
#define MLX90614_TOBJ_TH_INT_BITS		\
	(MLX90614_TOBJ_TH_HIGH_INT_BIT | MLX90614_TOBJ_TH_LOW_INT_BIT)

#define MLX90614_REG_TOBJ_TH_HIGH		0x06
#define MLX90614_REG_TOBJ_TH_LOW		0x07

/* scale in micro degrees Celsius */
#define MLX90614_TEMP_SCALE		31250
#define MLX90614_TEMP_TH_SCALE		500000

struct mlx90614_data {
	struct device *i2c;
	s16_t sample;

#ifdef CONFIG_MLX90614_TRIGGER
	struct device *gpio;
	struct gpio_callback gpio_cb;

	sensor_trigger_handler_t drdy_handler;
	struct sensor_trigger drdy_trigger;

	sensor_trigger_handler_t th_handler;
	struct sensor_trigger th_trigger;

#if defined(CONFIG_MLX90614_TRIGGER_OWN_THREAD)
	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_MLX90614_THREAD_STACK_SIZE);
	struct k_sem gpio_sem;
	struct k_thread thread;
#elif defined(CONFIG_MLX90614_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
	struct device *dev;
#endif

#endif /* CONFIG_MLX90614_TRIGGER */
};

#ifdef CONFIG_MLX90614_TRIGGER
int mlx90614_reg_read(struct mlx90614_data *drv_data, u8_t reg, u16_t *val);

int mlx90614_reg_write(struct mlx90614_data *drv_data, u8_t reg, u16_t val);

int mlx90614_reg_update(struct mlx90614_data *drv_data, u8_t reg,
		      u16_t mask, u16_t val);

int mlx90614_attr_set(struct device *dev,
		    enum sensor_channel chan,
		    enum sensor_attribute attr,
		    const struct sensor_value *val);

int mlx90614_trigger_set(struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

int mlx90614_init_interrupt(struct device *dev);
#endif

#endif /* _SENSOR_MLX90614_ */
