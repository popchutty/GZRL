.. SPDX-FileCopyrightText: 2026 GMaster contributors
.. SPDX-License-Identifier: Apache-2.0

Application-driven motor control
################################

This sample demonstrates the complete data path from a Zephyr motor device to
an independently composed controller and back to a CAN command. It is a
single-motor development application, not a complete robot or tuned controller.
It performs no automatic motor enable, brake release, or motion at boot.

The default RoboMaster Type C overlay describes an M3508/C620 endpoint with
motor ID 1 on CAN1. That default is a buildable topology example, not evidence
that this motor is connected or hardware-verified. Partial GM6020 hardware
results and remaining acceptance items are recorded in
``../../docs/motor_validation.rst``. Identify the actual motor and confirm the
test setup before running it.

Build and configuration
***********************

From the activated west workspace:

.. code-block:: console

   west build -p always -b robomaster_type_c \
     -d GZRL/build/motor-control GZRL/samples/motor_control

After the actual test fixture is connected and ready:

.. code-block:: console

   west flash -d GZRL/build/motor-control

Use the board's Micro-USB CDC ACM shell, not the debug probe's virtual serial
port. A terminal asserting DTR displays the ``motor:~$`` prompt. The default CAN
bitrate is 1 Mbit/s. Configure the actual drive's CAN timeout and verify that its
firmware stops output on communication loss before motion tests. A software
lease cannot stop an actuator if the application or CAN link has failed.

For another supported model, supply an application overlay changing
``sample_motor``'s ``model``, ``motor-id`` and, for DM MIT, ``feedback-id``. Keep
the ``motor0`` alias. This sample binds exactly one motor; do not add other
enabled GZRL motor nodes without also extending the application's device list.
The bus coordinator rejects partial ownership of a controller's motor nodes.

Runtime choices are Kconfig options under ``Motor sample runtime settings``:

* ``SAMPLE_MOTOR_EFFORT_LIMIT_MILLI`` is the maximum native command magnitude
  multiplied by 1000. The default 100000 means 100 command counts for the
  default M3508 profile. It is an application limit, not a motor rating.
* ``SAMPLE_MOTOR_GEAR_RATIO_MILLI`` is motor revolutions per output revolution
  multiplied by 1000. The default 1000 deliberately leaves values on the motor
  side; set the actual ratio before interpreting them as gearbox output values.
* ``SAMPLE_MOTOR_ENCODER_OFFSET`` specifies native encoder counts.
* DM MIT requires explicit firmware ``PMAX_MILLI``, ``VMAX_MILLI`` and
  ``TMAX_MILLI``. Defaults are zero, so changing only the model makes setup fail
  until the actual firmware limits and compatible effort limit are supplied.
  ``POSITION_WRAP_MILLI`` is optional verified native wrap period; zero selects
  no assumed wrapping. None of these values is inferred from a model name.
* ``SAMPLE_MOTOR_KP_MILLI``, ``KI_MILLI`` and ``KD_MILLI`` configure the single or
  inner PID. ``OUTER_KP_MILLI`` configures the outer position loop. Gains depend
  on the selected feedback units. The default proportional-only gains are
  demonstration values and must be tuned for the actual plant and mode.
* ``SAMPLE_MOTOR_SPEED_LIMIT_MILLI`` and ``POSITION_LIMIT_MILLI`` limit shell
  targets. Hardware drive parameters can impose additional restrictions.

The native simulator overlay is provided for build verification only. Its CAN
controller does not synthesize motor feedback, so it cannot demonstrate a
successful startup. Runtime adapter and controller tests live in the separate
test suites.

Authorization and ownership
****************************

The main thread exclusively owns the motor device APIs, controller state and
bus coordinator. Shell handlers put requests into an eight-entry bounded
``k_msgq``. ``motor stop`` uses an atomic stop request independent of queue
capacity; processing it discards queued operator commands. No shell command
directly modifies a PID or submits a CAN frame.

The loop targets a 1 ms period, measures the actual elapsed time, processes a
bounded number of receive frames, computes the selected control law and flushes
the bus. It resets controller history on mode transitions, startup completion,
stop and faults. The outer position loop runs approximately every 10 ms, using
its own measured interval. Logging, shell traffic and bus load still affect
timing; this is not a claim that arbitrary numbers of motors fit a 1 kHz bus
budget.

Control-thread reports and acknowledgments use Zephyr deferred logging, with
fixed buffering and thread blocking disabled. Keep this configuration when
using a slow console: synchronous status printing was observed to exceed the
PID's allowed time step at 115200 baud. Diagnostic congestion drops logs rather
than blocking control. ``motor info`` reports active-loop period extrema since
the last controller reset; temperature is meaningful only when the feedback
validity mask includes ``GZRL_MOTOR_VALID_TEMPERATURE``. These measurements do
not establish timing guarantees for other workloads.

Two distinct time limits are enforced:

* The sample's operator lease is 1000 ms by default. ``motor start`` grants a
  temporary authorization. A target command or ``motor refresh`` renews it.
  Repeated control-loop output does not renew it. Disconnecting a terminal or
  stopping refresh therefore stops the motor after lease expiry.
* The motor service requires fresh generated commands within 50 ms, fresh
  motion feedback within 100 ms, and bounds startup and transmission progress.

After expiry or a fault, ``refresh`` and target commands cannot restart the
motor. Inspect the cause and explicitly issue ``start`` again. Startup may
enable the drive or release its brake according to the selected profile;
``start`` is therefore a real hardware action even when its initial target is
zero. An invalid target revokes authorization. Unsupported drive modes are
rejected by the motor's capability checks rather than emulated silently.

Commands
********

All numeric shell arguments are **integers in milli-units**. For example,
``speed 500`` means 0.5 rad/s and ``direct 10000`` means 10 native effort units.
For DM MIT a native command effort unit is N m; for DJI, DM multi and MG it is
the documented raw command count. Feedback effort can use a different unit,
which is reported by ``info`` (0 = raw, 1 = mA, 2 = N m).

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Command
     - Behavior
   * - ``motor info``
     - Read feedback, validity, online/run/fault state and bus diagnostics.
   * - ``motor start``
     - Explicitly request startup; initialize the target to zero effort.
   * - ``motor stop``
     - Revoke authorization and request safe output; does not engage a brake.
   * - ``motor refresh``
     - Extend a current operator lease without changing its target.
   * - ``motor direct <effort>``
     - Apply a bounded native effort command.
   * - ``motor speed <speed>``
     - Software speed PID using output-shaft velocity feedback.
   * - ``motor angle <angle>``
     - Software single-turn rotor-angle PID using shortest-angle error and
       error differentiation. This target is on the motor side.
   * - ``motor position <position>``
     - Software output-displacement/velocity cascade; requires valid unwrapped
       position. Position is relative to the first sample or software zero.
   * - ``motor effort <feedback-effort>``
     - Software effort-feedback PID. Its target uses the feedback effort unit,
       while its output uses command units; gains must reflect that mapping.
   * - ``motor external_demo <target> <synthetic-feedback>``
     - Demonstrate an external-feedback controller input using a held synthetic
       value. Replace that input with an application sensor for a real loop.
   * - ``motor drive_speed <speed>``
     - MG internal output-shaft velocity loop.
   * - ``motor drive_single <angle> <max-speed> <clockwise:0|1>``
     - MG internal single-turn rotor-angle loop; maximum speed is output-side.
   * - ``motor drive_multi <angle> <max-speed>``
     - MG internal absolute output-angle loop; not the software-relative origin.
   * - ``motor status``
     - Request MG motion status while stopped.
   * - ``motor zero``
     - Reset software displacement origin while stopped with fresh feedback.
   * - ``motor clear_error`` / ``motor drive_zero``
     - Explicitly request a supported drive clear-error / hardware-zero command
       while stopped. Hardware zero changes the drive reference.
   * - ``motor brake``
     - Explicitly engage an MG mechanical brake while stopped.
   * - ``motor shutdown``
     - Request safe output, then stop CAN and detach. Reset to reopen the sample.

``info`` state values are 0 = stopped, 1 = starting, 2 = running, 3 = fault.
Only fields identified in the validity mask may be used as measurements.
Sending a maintenance frame successfully does not prove the drive accepted
its operation; inspect the corresponding feedback and physical behavior.

For a controlled speed test, issue ``start`` followed by a small configured
``speed`` target, inspect feedback and keep renewing authorization explicitly.
With the default one-second lease, interactive delays may expire the session;
a test harness can refresh at a bounded rate while actively monitoring the
fixture. It must stop refreshing when its user or feedback channel disappears.
Do not increase the lease merely to hide a missed control or monitoring path.

The sample waits at most 100 ms for pending safe-output delivery during explicit
shutdown. If delivery is unconfirmed, it reports that fact before stopping CAN;
the drive's configured communication timeout remains necessary. Mechanical
braking is never used as a generic substitute for a safe stopping trajectory.

Acceptance
**********

Record the actual board revision, motor and drive firmware, native limits,
gearbox ratio, command/current limits, control periods and tuned gains. Start
with observation, then verify explicit start/stop, a small bounded target,
operator lease expiry, feedback loss and restart authorization. Verify model
specific drive modes only on matching hardware. A successful build or a
synthetic feedback value is not a hardware acceptance result.
