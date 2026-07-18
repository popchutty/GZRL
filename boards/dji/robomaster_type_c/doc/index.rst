.. SPDX-FileCopyrightText: 2026 GMaster contributors
.. SPDX-License-Identifier: Apache-2.0

RoboMaster Development Board Type C
###################################

Overview
********

The DJI RoboMaster Development Board Type C is a robot-controller board based on
an STM32F407IGH6 Cortex-M4F microcontroller. This downstream board port targets
Zephyr v4.4.1 and uses upstream Zephyr drivers for the MCU peripherals, the
onboard BMI088 inertial sensor, and the onboard IST8310 magnetometer.

The hardware description follows, in priority order, the released Type C main
board schematic rev V1.0, the BMI088 FPC schematic rev V1.0, and the DJI Type C
user manual v1.0. The legacy GSRL CubeMX configuration is used only to recover
clocking and previously exercised protocol defaults when they agree with the
released hardware.

Hardware
********

* MCU: STM32F407IGH6 in UFBGA176
* CPU clock: 168 MHz from a 12 MHz HSE crystal
* Flash: 1 MiB internal flash
* RAM: 128 KiB main SRAM and 64 KiB CCM at ``0x10000000``
* Debug: external SWD probe; no onboard debugger or USB-to-UART bridge
* Default console: USB CDC ACM on the Micro-USB connector

The 64 KiB CCM region is exposed as ``zephyr,dtcm``. It is separate from normal
SRAM and must not be used for STM32F407 DMA buffers.

Supported board topology
************************

========================  ================================================
Function                  MCU connection
========================  ================================================
User RGB LED               TIM5 CH1/CH2/CH3 on PH10/PH11/PH12 (B/G/R)
User key                   PA0, active low
CAN1                       PD0 RX, PD1 TX, onboard TJA1044 transceiver
CAN2                       PB5 RX, PB6 TX, onboard TJA1044 transceiver
External USART1            PA9 TX, PB7 RX
External USART6            PG14 TX, PG9 RX
DBUS                       USART3 RX on PC11, after hardware inversion
Expansion SPI2             PB12 CS, PB13 SCK, PB14 MISO, PB15 MOSI
Expansion I2C2             PF1 SCL, PF0 SDA
Camera-control I2C1        PB8 SCL, PB9 SDA
Servo PWM 1--4             TIM1 CH1--4 on PE9/PE11/PE13/PE14
Servo PWM 5--7             TIM8 CH1--3 on PC6/PI6/PI7
Switched 5 V/laser output  TIM3 CH3 on PC8
Buzzer                     TIM4 CH3 on PD14
Battery monitor            ADC3 IN8 on PF10, 200 kohm/22 kohm divider
BMI088                     SPI1; CS PA4/PB0; IRQ PC4/PC5
IST8310                    I2C3 at 0x0e; reset PG6; data-ready PG3
BMI088 heater              TIM10 CH1 on PF6
USB FS                     PA11 DM, PA12 DP
========================  ================================================

Both CAN buses have onboard transceivers and fixed 120-ohm termination in the
released schematic. The board DTS limits each transceiver to 1 Mbit/s but does
not select a runtime bitrate. For the legacy RoboMaster configuration, set
``CONFIG_CAN_DEFAULT_BITRATE=1000000``.

The enclosure labels do not match the STM32 peripheral numbers: the connector
marked UART1 is connected to STM32 USART6, while the connector marked UART2 is
connected to STM32 USART1.

DBUS
****

The DBUS connector feeds USART3 RX through an onboard transistor inverter. The
board default is 100000 bit/s, 8 data bits, even parity, and one stop bit. The
signal seen by the MCU is already electrically inverted, so software must not
apply another inversion.

USB console and upper-computer protocols
****************************************

USB CDC ACM is the default Zephyr console, allowing basic samples to produce
output without an external USB-to-UART adapter. Applications that use USB for a
robot upper-computer protocol should disable the board console backend:

.. code-block:: cfg

   CONFIG_BOARD_SERIAL_BACKEND_CDC_ACM=n

The application can then reuse ``board_cdc_acm_uart`` as its dedicated transport
or replace it with its own USB function/composite-device definition. Do not
implicitly multiplex binary application traffic with the console stream.

Sensors and analog input
************************

The BMI088 is represented as separate accelerometer and gyroscope devices on
SPI1. Its defaults match the legacy board initialization: 800 Hz and +/-3 g for
the accelerometer, and 1000/116 Hz bandwidth with +/-2000 degrees/s for the
gyroscope. Zephyr v4.4.1's BMI08X implementation requires the asynchronous
sensor API for its RTIO bus, so the board supplies that Kconfig default whenever
the BMI08X driver is selected.

The IST8310 is available on I2C3 at address ``0x0e``. Zephyr v4.4.1's IST8310
binding does not model the board's PG6 reset or PG3 data-ready signals; those
pins are documented but intentionally not assigned invented properties.

The battery monitor uses ADC3 channel 8. The ideal input scaling is
``22 / (200 + 22)``, so 28 V at the board input produces approximately 2.775 V
at the ADC before component tolerance and reference error.

PWM safety
**********

The seven servo channels, switched laser output, passive buzzer, and IMU heater
are exposed through their PWM controllers. Their periods, duty-cycle limits,
and control policies are application responsibilities. Validate the laser and
heater only with suitable loads and bounded duty cycles.

The released schematic assigns PI7 to TIM8 CH3 as PWM channel 7. The legacy
CubeMX project temporarily named PI7 ``BUTTON_TRIG``; that application-specific
assignment is not carried into this board definition.

Camera limitation
*****************

The Type C camera connector routes an 8-bit DCMI bus plus I2C1. Zephyr v4.4.1
contains an STM32 DCMI driver, binding, and the package pinctrl definitions, but
its STM32F407 SoC DTS does not define the DCMI controller node. This board port
enables the I2C1 control bus but intentionally does not add an unverified
board-local DCMI controller. Camera support requires a separately validated
STM32F405/F407 SoC-level DCMI addition and an actual supported camera device.

Building
********

From the west workspace root:

.. code-block:: console

   west build -p always -b robomaster_type_c \
     -d GZRL/build/robomaster-type-c \
     zephyr/samples/hello_world

Flashing and debugging
**********************

OpenOCD with a CMSIS-DAP/DAP-Link probe is the default runner:

.. code-block:: console

   west flash
   west debug

The board SWD connector exposes SWDIO, SWCLK, ground, and 3.3 V reference.
Alternative runners are also registered:

.. code-block:: console

   west flash -r stm32cubeprogrammer
   west debug -r stlink_gdbserver
   west flash -r jlink
   west debug -r jlink

The STM32 ROM DFU bootloader is available through the Micro-USB connector after
setting BOOT0 high, BOOT1 low, and resetting the board:

.. code-block:: console

   west flash -r dfu-util

After flashing, set BOOT0 low again, keep BOOT1 low, and reset or power-cycle
before normal use so the MCU boots from internal flash.

Validation status
*****************

The port is build-validated against Zephyr v4.4.1. Physical validation must
record the tested board revision and verify the probes, USB CDC, both CAN buses,
external UARTs, DBUS, sensors, battery ADC, expansion buses, PWM outputs, buzzer,
laser output, and heater before the port is described as hardware-verified.
