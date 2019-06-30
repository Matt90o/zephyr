/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <kernel.h>
#include <drivers/sensor.h>

#include "mlx90614.h"

extern struct mlx90614_data mlx90614_driver;

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_DECLARE(MLX90614);

int mlx90614_attr_set(struct device *dev,
		    enum sensor_channel chan,
		    enum sensor_attribute attr,
		    const struct sensor_value *val)
{
	struct mlx90614_data *drv_data = dev->driver_data;
	s64_t value;
	u8_t reg;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	if (attr == SENSOR_ATTR_UPPER_THRESH) {
		reg = MLX90614_REG_TOBJ_TH_HIGH;
	} else if (attr == SENSOR_ATTR_LOWER_THRESH) {
		reg = MLX90614_REG_TOBJ_TH_LOW;
	} else {
		return -ENOTSUP;
	}

	value = (s64_t)val->val1 * 1000000 + val->val2;
	value = (value / MLX90614_TEMP_TH_SCALE) << 6;

	if (mlx90614_reg_write(drv_data, reg, value) < 0) {
		LOG_DBG("Failed to set attribute!");
		return -EIO;
	}

	return 0;
}

static void mlx90614_gpio_callback(struct device *dev,
				 struct gpio_callback *cb, u32_t pins)
{
	struct mlx90614_data *drv_data =
		CONTAINER_OF(cb, struct mlx90614_data, gpio_cb);

	gpio_pin_disable_callback(dev, CONFIG_MLX90614_GPIO_PIN_NUM);

#if defined(CONFIG_MLX90614_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_MLX90614_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

static void mlx90614_thread_cb(void *arg)
{
	struct device *dev = arg;
	struct mlx90614_data *drv_data = dev->driver_data;
	u16_t status;

	if (mlx90614_reg_read(drv_data, MLX90614_REG_STATUS, &status) < 0) {
		return;
	}

	if (status & MLX90614_DATA_READY_INT_BIT &&
	    drv_data->drdy_handler != NULL) {
		drv_data->drdy_handler(dev, &drv_data->drdy_trigger);
	}

	if (status & MLX90614_TOBJ_TH_INT_BITS &&
	    drv_data->th_handler != NULL) {
		drv_data->th_handler(dev, &drv_data->th_trigger);
	}

	gpio_pin_enable_callback(drv_data->gpio, CONFIG_MLX90614_GPIO_PIN_NUM);
}

#ifdef CONFIG_MLX90614_TRIGGER_OWN_THREAD
static void mlx90614_thread(int dev_ptr, int unused)
{
	struct device *dev = INT_TO_POINTER(dev_ptr);
	struct mlx90614_data *drv_data = dev->driver_data;

	ARG_UNUSED(unused);

	while (1) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		mlx90614_thread_cb(dev);
	}
}
#endif

#ifdef CONFIG_MLX90614_TRIGGER_GLOBAL_THREAD
static void mlx90614_work_cb(struct k_work *work)
{
	struct mlx90614_data *drv_data =
		CONTAINER_OF(work, struct mlx90614_data, work);

	mlx90614_thread_cb(drv_data->dev);
}
#endif

int mlx90614_trigger_set(struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	struct mlx90614_data *drv_data = dev->driver_data;

	gpio_pin_disable_callback(drv_data->gpio, CONFIG_MLX90614_GPIO_PIN_NUM);

	if (trig->type == SENSOR_TRIG_DATA_READY) {
		drv_data->drdy_handler = handler;
		drv_data->drdy_trigger = *trig;
	} else if (trig->type == SENSOR_TRIG_THRESHOLD) {
		drv_data->th_handler = handler;
		drv_data->th_trigger = *trig;
	}

	gpio_pin_enable_callback(drv_data->gpio, CONFIG_MLX90614_GPIO_PIN_NUM);

	return 0;
}

int mlx90614_init_interrupt(struct device *dev)
{
	struct mlx90614_data *drv_data = dev->driver_data;

	if (mlx90614_reg_update(drv_data, MLX90614_REG_CONFIG,
			      MLX90614_ALERT_EN_BIT, MLX90614_ALERT_EN_BIT) < 0) {
		LOG_DBG("Failed to enable interrupt pin!");
		return -EIO;
	}

	/* setup gpio interrupt */
	drv_data->gpio = device_get_binding(CONFIG_MLX90614_GPIO_DEV_NAME);
	if (drv_data->gpio == NULL) {
		LOG_DBG("Failed to get pointer to %s device!",
		    CONFIG_MLX90614_GPIO_DEV_NAME);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->gpio, CONFIG_MLX90614_GPIO_PIN_NUM,
			   GPIO_DIR_IN | GPIO_INT | GPIO_INT_LEVEL |
			   GPIO_INT_ACTIVE_HIGH | GPIO_INT_DEBOUNCE);

	gpio_init_callback(&drv_data->gpio_cb,
			   mlx90614_gpio_callback,
			   BIT(CONFIG_MLX90614_GPIO_PIN_NUM));

	if (gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb) < 0) {
		LOG_DBG("Failed to set gpio callback!");
		return -EIO;
	}

#if defined(CONFIG_MLX90614_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->gpio_sem, 0, UINT_MAX);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_MLX90614_THREAD_STACK_SIZE,
			(k_thread_entry_t)mlx90614_thread, dev,
			0, NULL, K_PRIO_COOP(CONFIG_MLX90614_THREAD_PRIORITY),
			0, 0);
#elif defined(CONFIG_MLX90614_TRIGGER_GLOBAL_THREAD)
	drv_data->work.handler = mlx90614_work_cb;
	drv_data->dev = dev;
#endif

	return 0;
}
