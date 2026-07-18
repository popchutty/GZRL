# SPDX-FileCopyrightText: 2026 GMaster contributors
# SPDX-License-Identifier: Apache-2.0

# keep first
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-type=sw")
board_runner_args(jlink "--device=STM32F407IG" "--speed=4000")
board_runner_args(dfu-util "--pid=0483:df11" "--alt=0" "--dfuse")

# keep first
include(${ZEPHYR_BASE}/boards/common/openocd-stm32.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stlink_gdbserver.board.cmake)
include(${ZEPHYR_BASE}/boards/common/dfu-util.board.cmake)
