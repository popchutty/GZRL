.. SPDX-FileCopyrightText: 2026 GMaster contributors
.. SPDX-License-Identifier: Apache-2.0

RoboMaster Type C validation shell
##################################

Overview
********

This sample provides a curated Zephyr Shell for validating the RoboMaster
Development Board Type C. It is a board-development image, not a production
robot application.

No hardware test runs automatically at boot. Open the board Micro-USB CDC ACM
terminal to use the Shell; all tests remain explicit operator commands.

The public command format is intentionally shallow:

.. code-block:: console

   test <peripheral-or-function> [mode and parameters]

Use ``test list`` for the complete test catalog and ``test --help`` for shell
help. Tab completes registered commands at the normal Zephyr shell prompt.

Safety scope
************

The shell deliberately does not expose arbitrary GPIO, PWM, CAN, watchdog,
memory, or device control. The image does not test or operate the laser,
BMI088 heater, watchdog reset, raw flash/settings, camera/DCMI, or USB host.
Do not connect a servo or other high-current load during the servo-signal
test.

The RGB, buzzer live-control, and unloaded-servo interactive sessions leave the
last valid setting active until it is changed, explicitly turned off, or the
session exits. Ctrl+C, ``quit``, ``pass``, ``fail``, and error exits run final
output cleanup. Disconnecting the CDC terminal does not generate Ctrl+C or end
one of these sessions; an active setting therefore remains active until the
operator reconnects and uses ``off`` or exits, or resets the board.

The buzzer score player is different: every accepted score has a finite event
count and bounded note duration. Playback progresses outside the blocked shell
input path and commands the buzzer inactive at natural completion, so a terminal
disconnection cannot leave a score sounding indefinitely.

Build, flash, and connect
*************************

From the west workspace root:

.. code-block:: console

   source ./activate-env.sh
   west build -p always -b robomaster_type_c \
     -d GZRL/build/type-c-validation \
     GZRL/samples/boards/robomaster_type_c_validation
   west flash -d GZRL/build/type-c-validation

Flash through the board's DAPLink/SWD interface. Open the separate board
Micro-USB CDC ACM serial port for the shell. The DAPLink virtual serial port
is not this shell console.

The serial settings are determined by USB CDC ACM. A terminal that asserts
DTR should display the normal Zephyr boot banner followed by the ``type-c:~$``
prompt. Run ``test list`` to discover the available tests or ``test quick`` to
begin.

Command reference
*****************

Discovery and suites
====================

``test info``
   Show the board target, Zephyr version, shell status, and excluded unsafe
   functions.

``test list [name]``
   Print every test with its command syntax, quick-suite membership,
   validation depth, and required setup. The optional name filters entries,
   for example ``test list can`` or ``test list rgb``.

``test quick``
   Run the objective no-extra-connection suite: clock cadence, curated device
   readiness, entropy, battery ADC, accelerometer, gyroscope, magnetometer,
   and CAN1 controller internal loopback.

``test quick guided``
   Run ``test quick`` and then guide the operator through the onboard RGB
   sequence, bounded buzzer tone, and user-button press. RGB and buzzer
   require an explicit ``yes``, ``no``, or ``skip`` verdict. A missing verdict
   is reported as ``NOT TESTED`` rather than success.

Single onboard checks
=====================

.. list-table::
   :header-rows: 1
   :widths: 32 68

   * - Command
     - Meaning
   * - ``test clock``
     - Check that the configured 168 MHz clock and kernel timing cadence agree.
   * - ``test devices``
     - Check initialization of curated onboard controllers and devices. This is
       readiness evidence only; connector pins are not exercised.
   * - ``test entropy``
     - Perform two guarded entropy reads and confirm the returned buffers differ.
   * - ``test battery``
     - Read the onboard battery voltage-divider sensor. Near zero is valid when
       no battery input is connected.
   * - ``test accel``
     - Read one BMI088 accelerometer XYZ sample.
   * - ``test accel interactive``
     - Report changing BMI088 accelerometer axes at a bounded rate until Ctrl+C.
   * - ``test gyro``
     - Read one BMI088 gyroscope XYZ sample.
   * - ``test gyro interactive``
     - Report changing BMI088 gyroscope axes at a bounded rate until Ctrl+C.
   * - ``test magn``
     - Read one IST8310 magnetometer XYZ sample.
   * - ``test magn interactive``
     - Report changing IST8310 magnetometer axes at a bounded rate until Ctrl+C.
   * - ``test i2c``
     - Check I2C1 and I2C2 controller readiness without starting a transaction.
   * - ``test can internal``
     - Send, receive, and compare one frame through CAN1 controller internal
       loopback. This does not test the transceiver, connector, termination, or bus.

Move or rotate the board and repeat the sensor commands to confirm that the
reported values respond plausibly. A successful one-sample API read is not a
complete calibration or accuracy test.

The ``interactive`` sensor commands print an initial XYZ baseline and then print
only axes whose converted values changed. Sampling is limited to approximately
10 Hz so the terminal remains readable; the magnetometer also tolerates a
bounded conversion-not-ready interval. Ctrl+C ends the monitor. A session that
observes a changed value is ``SUCCESS``; canceling before any changed value is
``NOT TESTED``. These modes exercise driver reads and observable variation, not
calibration, axis alignment, magnetic compensation, or accuracy.

Guided onboard checks
=====================

``test rgb``
   Show green, red, blue, and low-brightness white, then request an operator
   verdict. All RGB channels are commanded off before returning.

``test button [timeout_ms]``
   Wait for one user-button active edge. The default is 10000 ms; the accepted
   range is 100 to 30000 ms. No press within the interval is ``NOT TESTED``.

``test buzzer``
   Generate a bounded 4 kHz, 25 percent-duty tone for one second, silence the
   output, and request an operator verdict.

Fixture checks
==============

``test uart 1``
   Jumper PA9 TX to PB7 RX on the connector marked UART2. The command applies
   115200 8N1, exchanges a fixed byte pattern, and restores the previous UART
   configuration.

``test uart 6``
   Jumper PG14 TX to PG9 RX on the connector marked UART1. The command applies
   115200 8N1, exchanges a fixed byte pattern, and restores the previous UART
   configuration.

``test spi2``
   Jumper PB15 MOSI to PB14 MISO. The command compares transfers at 500 kHz
   and 4 MHz. Chip select is not exercised.

``test can physical``
   Connect CAN1 H/L/GND to CAN2 H/L/GND. Do not add termination: the board
   already provides termination on both interfaces. The command exchanges
   1 Mbit/s frames in both directions and then stops both controllers in
   normal mode.

A missing or incorrect fixture normally produces ``ERROR`` with the failed
stage or driver errno; it is not treated as a board pass.

Interactive modes
*****************

The RGB, buzzer, buzzer-score, and servo sessions consume complete input lines
through Zephyr ``shell_readline()``. Zephyr disables Tab completion and command
history while this nested input capture is active, so enter ``help`` to display
the session grammar. The validation sessions use ``pass`` and ``fail`` for
operator or instrument evidence. The score player is an entertainment utility
and instead uses ``stop`` and ``quit``. Ctrl+C cancels, and cleanup runs for every
exit path. The button and sensor monitors report state changes and use Ctrl+C as
their only exit.

RGB
===

.. code-block:: console

   test rgb interactive

Inside the session:

.. code-block:: text

   set <red 0-100> <green 0-100> <blue 0-100>
   off
   status
   help
   pass | fail | quit

The last valid setting remains active until another setting replaces it,
``off`` is entered, or the session exits.

Button
======

.. code-block:: console

   test button interactive

The shell reports debounced logical pressed/released transitions until Ctrl+C.
If no state change is observed, the session ends as ``NOT TESTED``.

IMU sensors
===========

.. code-block:: console

   test accel interactive
   test gyro interactive
   test magn interactive

Each command prints the initial XYZ sample and then reports changed axes with
six decimal places and the same units as the one-shot command. Move or rotate
the board to generate changes. Press Ctrl+C to stop; completed input lines are
ignored. These commands are intentionally observation-only and do not claim
sensor calibration or accuracy.

Zephyr v4.4.1 does not provide a generic maintained attitude-estimation API that
fuses an arbitrary accelerometer, gyroscope, and magnetometer into Euler angles
and a quaternion. The sensor and sensing APIs provide data plumbing/frameworks,
and CMSIS-DSP provides quaternion math primitives, but neither supplies the
requested AHRS algorithm. Zephyr's ST LSM6DSV SFLP support is a hardware-specific
six-axis game-rotation-vector output and does not apply to this board's BMI088
plus IST8310 topology or provide the requested Euler output. Consequently this
sample does not add an attitude command or pretend to test an algorithm that
Zephyr does not provide.

Buzzer
======

.. code-block:: console

   test buzzer interactive

Inside the session:

.. code-block:: text

   set <frequency 100-5000 Hz> <duty 1-50 percent>
   off
   status
   help
   pass | fail | quit

The last valid tone remains active until another setting replaces it, ``off``
is entered, or the session exits.

Buzzer score player
===================

.. code-block:: console

   test buzzer score

This Easter-egg utility is intentionally absent from ``test list`` and does not
produce validation evidence. It configures the sample-local TIM4 clock for the
full piano MIDI range, then accepts one complete monophonic score per input line.
Paste a prepared line and press Enter to begin playback.

The grammar is:

.. code-block:: text

   score    := T<BPM> <event> [<event> ...]
   event    := <MIDI>/<duration>[.] | R/<duration>[.]
   BPM      := 40..240
   MIDI     := 21..108
   duration := 1 | 2 | 4 | 8 | 16

``T120`` sets 120 quarter-note beats per minute. MIDI notes use standard
twelve-tone equal temperament: ``60`` is C4, ``61`` is C#4/Db4, and ``69`` is
A4 at 440 Hz. ``R`` is a rest. Durations ``1``, ``2``, ``4``, ``8``, and ``16``
mean whole, half, quarter, eighth, and sixteenth notes. A trailing period makes
the duration dotted, for example ``60/4.``. Separate tokens with one or more
spaces or tabs; whitespace is not allowed inside an event.

For example:

.. code-block:: text

   T120 67/8 67/8 69/4 67/4 72/4 71/2 R/8

A score is limited to 511 characters and 100 events. The buzzer uses 25 percent
duty. Each sounding note is gated for seven eighths of its duration followed by
a one-eighth silent articulation gap, so repeated notes remain distinct. Rests
remain inactive for their full duration. The entire candidate is checked before
it can replace an active score.

Inside the session:

.. code-block:: text

   <paste a score line>
   status
   stop
   help
   quit

``stop`` immediately silences playback but remains in the session. Pasting a
new valid score stops and replaces the previous one. ``quit`` and Ctrl+C stop
playback, perform final cleanup, and exit. The complete format and limits also
appear in ``test buzzer score --help`` and the mode's ``help`` output.

The board Devicetree sets TIM4 to a 1 MHz counter clock and exposes the buzzer
through the ``pwm-buzzer`` alias. This allows MIDI 21-108 on the 16-bit timer
while retaining the existing 100-5000 Hz interactive range and bounded 4 kHz
test. Score mode verifies the timer rate before accepting input rather than
silently playing at the wrong pitch.

Unloaded servo signal
=====================

.. code-block:: console

   test servo unloaded <1-7>

Inside the session:

.. code-block:: text

   pulse <1000-2000 microseconds>
   off
   status
   help
   pass | fail | quit

This command generates only a 20 ms-period PWM signal for measurement. Do not
attach a servo or load. The selected pulse remains active until it is replaced,
``off`` is entered, or the session exits. All seven servo outputs are commanded
inactive before entry, before selecting a new output, after errors, and at final
cleanup. Use an oscilloscope or logic analyzer and a common ground to provide a
meaningful ``pass`` verdict.

Result model
************

Execution outcome and validation depth are separate.

``SUCCESS``
   The check met its expectation at the printed depth.

``ERROR``
   Setup, a driver/API call, comparison, required response, or cleanup failed.
   The output includes the concrete reason and errno when available.

``NOT TESTED``
   Required operator or fixture evidence was not obtained. This is not a pass.

``INFO``
   Measurements, setup instructions, progress, and coverage limitations.

Validation depths include ``readiness``, ``functional``,
``internal-loopback``, ``physical-fixture``, ``operator-observation``, and
``instrumentation``. For example, a ``SUCCESS`` at ``readiness`` confirms
initialization only, while ``physical-fixture`` includes the documented
external path.

Every valid test command ends with one readable aggregate result. Run
``retval`` immediately afterward to inspect the Zephyr Shell return value.
Completed success returns zero. Driver failures retain their negative errno
where possible, logical mismatches return ``-EIO``, and incomplete/cancelled
required evidence returns a nonzero incomplete result.

I2C target access
*****************

The standard Zephyr I2C shell remains available only for an operator-connected,
known 3.3 V target with a documented protocol. ``test i2c`` performs readiness
checks only. Do not blindly scan an unknown bus: probing addresses can disturb
attached devices or cause side effects.

Validation boundaries
*********************

Build or Twister success proves that the image integrates and compiles; it is
not physical board validation. Likewise:

- readiness does not exercise pins or connectors;
- internal CAN loopback does not exercise the transceiver or physical bus;
- PWM API success does not prove measured waveform frequency or duty;
- RGB and buzzer behavior require an operator verdict;
- sensor API reads do not prove calibration or accuracy;
- RTC validation is not included, and this sample must not reset the backup
  domain.
