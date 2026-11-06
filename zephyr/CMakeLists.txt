# SPDX-FileCopyrightText: 2026 GMaster contributors
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(../include)

if(CONFIG_GZRL_CONTROL)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/control ${CMAKE_CURRENT_BINARY_DIR}/control)
endif()

if(CONFIG_GZRL_MOTOR_CORE)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/motor ${CMAKE_CURRENT_BINARY_DIR}/motor_core)
endif()

if(CONFIG_GZRL_MOTOR)
  zephyr_library_named(gzrl_motor_adapter)
  zephyr_library_sources(../drivers/motor/motor_can.c ../subsys/motor/motor_bus.c)
endif()
