# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_MLX90614 mlx90614.c)
zephyr_library_sources_ifdef(CONFIG_MLX90614_TRIGGER mlx90614_trigger.c)
