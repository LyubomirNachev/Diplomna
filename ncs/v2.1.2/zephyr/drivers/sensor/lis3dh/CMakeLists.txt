# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(lis3dh.c)
zephyr_library_sources_ifdef(CONFIG_LIS3DH_TRIGGER lis3dh_trigger.c)
