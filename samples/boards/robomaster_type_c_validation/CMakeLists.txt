# SPDX-FileCopyrightText: 2026 GMaster contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(robomaster_type_c_validation)

target_sources(app PRIVATE
  src/validation.cpp
  src/onboard_checks.cpp
  src/fixture_checks.cpp
  src/buzzer_score.cpp
  src/shell_commands.cpp
)
