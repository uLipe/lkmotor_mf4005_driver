# Copyright (c) 2024 Felipe Neves
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_LKMOTOR_MF4005_DRIVER)

zephyr_interface_library_named(MF4005)
zephyr_include_directories(include)

zephyr_library()
zephyr_library_sources(drivers/lkm_mf4005_motor.c)
zephyr_library_sources_ifdef(CONFIG_MF4005_DRIVER_SHELL drivers/lkm_mf4005_motor_shell.c)

zephyr_library_link_libraries(MF4005)
target_link_libraries(MF4005 INTERFACE zephyr_interface)
endif()