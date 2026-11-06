# SPDX-FileCopyrightText: 2026 GMaster contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
project(gzrl_portable_tests LANGUAGES C CXX)

get_filename_component(GZRL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

add_library(gzrl_portable STATIC
  ${GZRL_ROOT}/lib/control/pid.c
  ${GZRL_ROOT}/lib/control/filter.c
  ${GZRL_ROOT}/lib/motor/core.c
  ${GZRL_ROOT}/lib/motor/grouped.c
  ${GZRL_ROOT}/lib/motor/mit.c
  ${GZRL_ROOT}/lib/motor/lkmg.c
)
target_include_directories(gzrl_portable PUBLIC ${GZRL_ROOT}/include)
target_link_libraries(gzrl_portable PUBLIC m)
set_target_properties(gzrl_portable PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)

add_executable(gzrl_portable_smoke src/c_api.c src/cpp_api.cpp)
target_link_libraries(gzrl_portable_smoke PRIVATE gzrl_portable)
set_target_properties(gzrl_portable_smoke PROPERTIES
  C_STANDARD 17 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
  CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO
)

foreach(target gzrl_portable gzrl_portable_smoke)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -Wdouble-promotion)
endforeach()

enable_testing()
add_test(NAME gzrl.portable.c_cpp COMMAND gzrl_portable_smoke)
