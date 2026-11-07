.. SPDX-FileCopyrightText: 2026 GMaster contributors
.. SPDX-License-Identifier: Apache-2.0

Motor migration validation record
#################################

Software acceptance: 2026-11-07
*******************************

Baseline: workspace Zephyr v4.4.1, native_sim and robomaster_type_c. No physical
controller or motor was flashed or operated during this software acceptance.

.. list-table::
   :header-rows: 1

   * - Check
     - Result
   * - Portable control Ztests
     - 12/12 passed, including simulated closed-loop convergence.
   * - Portable motor protocol Ztests
     - 14/14 passed across all seven model profiles.
   * - Fake CAN device/lifecycle tests
     - 28/28 passed, including C++ API linkage.
   * - Native Zephyr CAN loopback
     - 1/1 passed, exercising the real loopback driver's asynchronous callbacks.
   * - Unified Twister test tree
     - 4/4 configurations, 55/55 cases, zero warnings.
   * - Independent C17/C++11 build without Zephyr
     - GCC and Clang builds passed; CTest 1/1 passed with each toolchain.
   * - Module smoke
     - Built and ran; expected GZRL message observed.
   * - Motor control sample
     - Native simulator and Type C builds passed.
   * - Existing Type C board validation sample
     - Build passed; physical checks remain separate.
   * - Native motor Shell observation
     - Boot STOPPED with TX=0; missing-feedback startup timed out; refresh/direct
       could not reauthorize; CAN shutdown and detach succeeded.
   * - clang-tidy
     - All eight new production library/driver/subsystem C sources checked using
       one complete host/llvm compile database, warnings treated as errors.
   * - Formatting, licensing and whitespace
     - clang-format, REUSE and git diff checks passed.

Regression evidence
*******************

Tests first reproduced the following failures, then passed after their causal
fixes. These tests remain in the fake CAN suite:

* Startup could accept feedback decoded before the request in the same clock tick.
* Startup could accept an undrained frame received before authorization.
* Synchronous send failures could exhaust neither the safe-output attempt budget
  nor its lifetime; mailbox contention also needed a safe-output deadline.
* A later healthy packet in one queue drain could overwrite a drive fault or
  unexpected disable indication.
* Fresh feedback arriving after an offline interval could mask the interruption
  and leave the old running permission valid.

Receive serials and startup feedback evidence close the first two cases. Safe
operations consume a finite retry budget and expire. Accepted packets latch
drive faults, unexpected disable/brake and observed feedback gaps before any
later packet can replace their evidence.

Reproduction
************

From the workspace root after sourcing ``activate-env.sh``:

.. code-block:: console

   west twister -T GZRL/tests -p native_sim -j 4 \
     -O GZRL/twister-out-tests --inline-logs
   west twister -T GZRL/samples -p native_sim -p robomaster_type_c -j 4 \
     -O GZRL/twister-out-samples --inline-logs
   cmake -S GZRL/tests/portable -B GZRL/build/portable -DCMAKE_BUILD_TYPE=Release
   cmake --build GZRL/build/portable
   ctest --test-dir GZRL/build/portable --output-on-failure

The reports are local build artifacts, not committed source. In a restricted
execution environment the native 32-bit simulator and Python multiprocessing
runner can require permission to execute; this is distinct from a test failure.
REUSE can run with ``--no-multiprocessing`` when local process creation is restricted.

Hardware acceptance: partial, 2026-11-07
****************************************

GM6020 communication, bounded direct control and lifecycle checks passed on the
fixture below. Positional and incremental PI and cascaded position control were
exercised. Small-angle tracking improved in the follow-up below, but a general
tracking-accuracy specification is **NOT ACCEPTED**. M3508/C620, M2006/C610,
DM4310 MIT, DM2325 MIT, Damiao one-to-four and LK MG remain **NOT TESTED** on physical hardware.
Builds, simulation and internal CAN loopback do not establish a motor hardware pass.

Fixture and settings
====================

* RoboMaster Type C, STM32F407IGH6, connected through Horco CMSIS-DAP (probe
  firmware 2.1.0); CPUID 0x410fc241 and DBGMCU IDCODE 0x10076413.
* User-supplied GM6020, DJI ID 2, CAN1 at 1 Mbit/s; USART6 (enclosure UART1)
  connected to the probe's serial bridge at 115200 8N1.
* GM6020 voltage-command profile, control ID 0x1ff, bytes 2--3 for ID 2;
  feedback ID 0x206. Gear ratio 1, encoder offset 0. Effort feedback remains raw
  counts, not an inferred physical current or torque.
* Board independent 24 V input confirmed by the user. Board revision, motor
  firmware, measured motor supply voltage, termination and drive communication
  watchdog settings were not independently established.
* Operator lease 500 ms, generated-command timeout 50 ms, feedback timeout
  100 ms, startup timeout 500 ms. Requested control period 1 ms.
* All transmitted commands were bounded by a configured limit of at most
  +/-2000 raw counts. Direct-control tests used magnitudes 50, 100, 200, 400 and
  800. Later low-gain closed-loop tests used a +/-1200 limit.

Observed results
================

.. list-table::
   :header-rows: 1

   * - Check
     - Result and scope
   * - Flash and console
     - OpenOCD programming succeeded. Each subsequent test image was read back
       in full with pyOCD while halted and compared byte-for-byte to zephyr.bin.
       USART6 boot output and bidirectional Shell communication worked.
   * - Initial permission
     - STOPPED, authorization false, motor-service TX count zero; feedback can
       be observed without enabling motion.
   * - CAN1 internal path
     - Combined loopback and listen-only mode completed an asynchronous send and
       received ID 0x123 with eight zero bytes. This did not test the external bus.
   * - Board supply diagnosis
     - With only probe power, no external frames arrived; one all-zero, one-shot
       0x1ff send failed with bit0 errors and TEC 136. The user confirmed that
       the board XT30 input was disconnected. Connecting independent 24 V restored
       approximately 1000 feedback frames/s without a firmware change.
   * - Feedback
     - online=1, valid=0x1f: angle, relative position, velocity, effort and
       temperature valid. No RX queue drops were observed in successful runs.
       Temperature read 26--28 degrees C during the recorded tests.
   * - Start, direct command, stop
     - Zero-command startup/stop passed. Initial +800 pulse moved approximately
       19 mrad; a later -800 pulse moved approximately -56 mrad. Both directions
       produced signed angle/velocity responses, followed by zero reported speed
       after explicit stop. All successful-run sends completed without failures.
   * - Authorization expiry
     - +800 stopped after the 500 ms lease expired. Subsequent refresh and direct
       requests returned -EACCES and could not restart; explicit start was required.
   * - Controller communication interruption
     - While running with zero command, CAN1 was stopped through Zephyr CAN Shell.
       State became FAULT/BUS, online=0 and authorization=0. Restarting CAN restored
       feedback but retained the fault and revoked permission. Explicit motor start
       was required. This was controller-stop injection, not physical cable removal.
   * - Position continuity
     - The communication gap cleared the multi-turn position-valid bit (0x1f to
       0x1d). Reconnection and explicit start did not silently restore it. After
       stopping, explicit software zero restored position=0 and valid=0x1f.
   * - Control timing after logging fix
     - Active-loop intervals reported 900--1100 us, using a 100 us clock resolution,
       during repeated status queries and closed-loop tests. This is a measurement
       of this single-motor fixture, not a general worst-case execution-time bound.

Closed-loop observations and limitations
========================================

An initial speed-loop experiment used Kp=3000, Ki=4000, Kd=0 and a +/-2000
output limit. It oscillated; a sampled speed of -2.199 rad/s triggered the host
script's 1.5 rad/s guard, which requested stop. Zero speed was then confirmed.
These gains were discarded and are not sample defaults or recommended tuning.

A second experiment used Kp=300, Ki=1500, Kd=0 with a +/-1200 limit and
+/-0.3 rad/s targets. Both directions responded and stopped normally. Static
friction and quantized RPM feedback affected low-speed tracking. Positive-run
samples after breakaway ranged from 0.104 to 0.418 rad/s; negative-run samples
included pauses and a -0.628 rad/s sample. These observations demonstrate the
software-loop data path, not an accepted speed-regulation specification.

The cascade used the same inner gains and outer Kp=4, with a 0.5 rad/s outer
limit. A 0.1 rad displacement target reached 0.164 rad at its peak and was at
0.161 rad after stopping the approximately four-second trial. The 0.061 rad
final position error means position accuracy is **NOT ACCEPTED**. Plant-specific
controller tuning remains necessary; no accuracy or load-handling guarantee is
made for these experimental gains.

PID follow-up: 2026-11-07
=========================

All follow-up runs used the same fixture, a +/-1200 command limit and a 500 ms
operator lease. The portable algorithms and production sample were unchanged.
A temporary sample copy recorded per-control-tick velocity/output extrema,
window averages and PID terms through deferred logging. It also stopped on
feedback speed above 1.5 rad/s or displacement above 2.5 rad from software zero.
The host imposed smaller travel limits for position trials. These guards did
not trigger. They are test instrumentation, not production protection guarantees.

Six-second speed trials produced the following mean reported velocities over
approximately the final two seconds. Gains use the explicit-seconds convention
of :doc:`control`; output is in native command counts. Kd was zero throughout,
so these are PI tests, not physical validation of derivative tuning.

.. list-table::
   :header-rows: 1

   * - PID form
     - Kp / Ki
     - Target (rad/s)
     - Mean feedback (rad/s)
   * - Positional
     - 500 / 1000
     - +0.300 / -0.300
     - +0.303 / -0.300
   * - Positional
     - 1000 / 1000
     - +0.300 / -0.300
     - +0.262 / -0.275
   * - Incremental
     - 500 / 1000
     - +0.300 / -0.300
     - +0.287 / -0.239

These averages do not establish instantaneous regulation accuracy or equivalence
between tuning profiles: starting rotor angles differed, breakaway was delayed
and speed rippled. The positional 500/1000 positive trial had a whole-trial
feedback range of -0.104 to +0.733 rad/s. Raising Kp to 1000 increased its
positive-trial peak to 1.047 rad/s. RPM quantization and disagreement between
short-window speed averages and encoder displacement slopes also preclude
treating the table as independently measured shaft-speed accuracy.

A 12-second +0.100 rad cascade trial with inner Kp=500, Ki=1000 and outer Kp=2
ended at +0.213 rad, with a sampled peak of +0.221 rad. Recorded integral terms
showed continued positive output after passing the target, followed by a slow
reversal while the shaft remained almost stationary. That profile was discarded.

Increasing the inner Kp to 1000 and outer Kp to 6, retaining inner Ki=1000,
Kd=0 and a 0.5 rad/s outer limit, gave the following four 12-second trials.
Both loops used positional PID; the outer loop had Ki=Kd=0 and updated at
approximately 10 ms. Software zero and controller reset preceded each trial.

.. list-table::
   :header-rows: 1

   * - Trial
     - Target (rad)
     - Active final position (rad)
     - Sampled overshoot (rad)
     - Position after zero-output stop (rad)
   * - Positive, first rotor region
     - +0.100
     - +0.099
     - 0.000
     - +0.097
   * - Negative, first rotor region
     - -0.100
     - -0.112
     - 0.018
     - -0.113
   * - Positive, second rotor region
     - +0.100
     - +0.101
     - 0.001
     - +0.095
   * - Negative, second rotor region
     - -0.100
     - -0.099
     - 0.000
     - -0.094

Active final absolute errors were approximately 0.001--0.012 rad, using
milliradian-resolution status output. These are
sampled observations, not a settling-time, repeatability or loaded-accuracy
specification. Zero-output stop releases the holding command, so its position
must be distinguished from active closed-loop position. Do not install these
experimental gains as general sample defaults.

Incremental PI lease expiry was also exercised with a nonzero integral. In the
completed repeat, expiry cleared approximately -806.5 command counts of integral
to exactly zero and revoked authorization. Refresh returned -EACCES; explicit
restart entered idle with zero integral/output. Six subsequent zero-speed
refresh/status cycles stayed running without meaningful displacement, followed
by an explicit stop.

An earlier host assertion requiring integral/output to remain exactly zero
throughout the restarted closed loop was invalid: occasional -1 RPM feedback
created new proportional output and about 0.314 counts of integral. Both expiry
and idle restart had already cleared the old history. The corrected check kept
those exact reset assertions, then bounded newly accumulated integral below
10 counts, output below 100 counts and displacement within 0.005 rad during
zero-speed control. It passed without any algorithm change. The original
assertion failure and its feedback evidence are retained in the local record.

Across the follow-up, computed output peaked at approximately 1166 counts,
below the configured 1200 and the user's 2000 ceiling. There were no RX drops,
send failures or transmit stalls; temperature was 27--28 degrees C. Recorded
control intervals ranged from 800 to 1200 us with the temporary instrumentation.
All runs ended stopped. This does not validate driver watchdog behavior during
a physical cable fault, derivative tuning, or operation with a different load.

Hardware-discovered sample fix
==============================

Before the fix, synchronous status printing from the control thread at 115200
baud could make a zero-speed PID update return -ERANGE and stop. The failure
occurred during consecutive motor info requests, with no RX drops or CAN error.

Control-thread status, acknowledgments and fault notices now use Zephyr deferred
logging with fixed storage and thread blocking disabled. A one-message wakeup
threshold preserves interactive response. Shell handlers still enqueue requests;
only the control thread accesses motor and controller state. PID dt validation
and timeout protections are unchanged. The added period extrema are reset with
controller history; temperature must be interpreted with its feedback-valid bit.

Repeating twelve consecutive zero-speed status/refresh cycles passed with
900--1100 us intervals, no stop and no RX drops. Nonzero speed and cascade
experiments subsequently retained the same measured interval range. Logging
congestion may drop diagnostics instead of delaying the control thread.

Remaining acceptance
====================

* Specified speed and position accuracy across operating points and loads,
  derivative tuning, single-turn wrapped-angle hardware control, larger/multiple-turn
  travel and external sensor feedback.
* Physical cable disconnection during motion and actual motor-firmware watchdog
  stop timing. The controller-stop test with zero output does not replace these.
* Drive firmware identity, watchdog configuration, termination and board revision.
* All other motor models and board peripherals outside this fixture.

The final board image retains the stopped Shell application, USART6 and GM6020
ID 2 topology. Its command limit is reduced to +/-800, with the sample's original
Kp=1, Ki=0, Kd=0 and outer Kp=1 defaults. Experimental PI gains and the unrestricted
CAN diagnostic Shell are removed. Boot and explicit startup use zero command;
no automatic restart or motion is configured. Local build artifacts retain the
final fixture configuration and compact observations; temporary host test scripts
are removed after validation.

Final image and engineering checks
==================================

The final 81900-byte image passed full flash readback comparison. Its SHA-256 is
``2cc298accbf9fe39663c177ca3dd083938420fb8cf7806087108fd427698ea6f``.
A request for 801 raw counts was rejected with -ERANGE and revoked authorization,
confirming the final +/-800 limit. Six further zero-speed status/refresh cycles
passed. The first-session final observed state was STOPPED, online, authorization false,
valid=0x1f, zero speed, zero RX drops and all 1136 submitted frames completed.

After the PID follow-up, the same 81900-byte image and hash were restored and
again passed full flash readback comparison. The final observation was STOPPED,
online, authorization false, valid=0x1f, zero speed and TX=0 since reboot, with
temperature 28 degrees C. Experimental gain profiles, telemetry and guards were
removed together with temporary host scripts; no production algorithm changes
were needed. Follow-up logs, measurements and a response plot are retained under
``build/gm6020-validation/observations/pid-followup/``. Formatting, REUSE and
whitespace checks were repeated for the documentation-only repository change.

The affected sample passed both Twister build-only configurations (native_sim
and robomaster_type_c), with zero warnings. A complete host/llvm sample compile
database covered all eight production library/driver/subsystem C files plus
the sample. All nine passed clang-tidy with warnings treated as errors; the
upstream Shell macros' identical-NULL-branch false positives are suppressed only
around command registration. Formatting, REUSE and whitespace checks passed.
The upstream Zephyr worktree remained clean.

Local reproducibility artifacts are in ``build/hardware-fixtures/`` (final
overlay and configuration) and ``build/gm6020-validation/observations/`` (logs
and compact measurements). They are ignored build artifacts. The fixture
overlay selects ``model = "gm6020"``, ``motor-id = <2>`` and routes both
``zephyr,console`` and ``zephyr,shell-uart`` to ``&usart6``. Its configuration
uses ``SAMPLE_MOTOR_EFFORT_LIMIT_MILLI=800000``, ``SAMPLE_MOTOR_LEASE_MS=500``,
disables ``BOARD_SERIAL_BACKEND_CDC_ACM`` and serial DTR checking, and enables
``SERIAL``, ``CONSOLE``, ``UART_CONSOLE`` and ``SHELL_BACKEND_SERIAL``.
It does not enable CAN Shell or override PID gains.

DM-J4310-2EC and shared CAN1: 2026-11-08
****************************************

The user added a powered DM-J4310-2EC to the existing CAN1 bus alongside the
GM6020. A temporary read-only probe was flashed and verified byte-for-byte
(60156 bytes, SHA-256
``a2d69a80089fda3b51f5df2027a9e1e7e075e5039f716a75c48c8acd7018d3ca``).
It sent only documented register-read requests: standard ID 0x7ff, DLC 4,
payload ``CANID_L CANID_H 0x33 RID``. No enable, motion, parameter-write,
clear-error or hardware-zero command was sent to the DM motor.

The user's DM-J4310-2EC V1.2 manual, V1.4 dated 2026-09-14, was the protocol
reference (pages 15--18). Read replies established:

.. list-table::
   :header-rows: 1

   * - Register
     - Observed value
   * - ESC_ID / MST_ID
     - 0x002 / 0x005; confirmed through addressed replies and repeated MST_ID reads.
   * - CTRL_MODE
     - 1 (MIT).
   * - PMAX / VMAX / TMAX
     - Approximately 3.1415925 rad / 30 rad/s / 10 N m; these are mapping ranges.
   * - Gr / GREF
     - 10 / 1.
   * - TIMEOUT
     - 0; a working drive communication watchdog has not been established.
   * - OC_Value / KT_Value
     - Approximately 0.8 / 0; no settings were changed.
   * - sw_ver
     - Raw uint32 0x37313035; not interpreted as a validated release identity.

The DM node number 2 does not itself conflict with DJI motor number 2: their
control/feedback arbitration IDs are 0x002/0x005 and 0x1ff/0x206 respectively.
The geared dual-encoder variant's axis semantics and additional drive fault
states still need checking before applying the existing DM4310 profile.

Initially CAN was error-active with TEC=REC=0 and approximately 1000 GM6020
feedback frames per second. At probe uptime 51.699 s it reported error-passive,
TEC=0, REC=255, DJI RX=51669 and queue drops=0. Subsequent read requests for
sub_ver and MAX_SPD received no replies in their observation windows. Further
requests were stopped and the user was asked to check wiring, connectors,
power and CAN termination before restarting read-only checks. The physical
cause has not been established; no workaround or protocol change was made.

After the user reversed the suspected DM CAN H/L pair, a read-only recheck at
probe uptime 642--645 s found error-active CAN with TEC=REC=0 and GM6020 RX
increasing at approximately 1000 frames/s. However, all seven requests to DM
node 2 (MST_ID, ESC_ID, TIMEOUT, CTRL_MODE, PMAX, VMAX and TMAX) had no replies
in the observation windows. Error counters returning to zero therefore did not
establish DM connectivity. Motion testing remained paused; the user was asked
to verify terminal labels, connector seating, DM power/indicator state and
termination before another check. No drive settings or IDs were changed.

Restoring the original H/L order restored valid node-2 replies. A second
seven-request check addressed only node 2: six registers replied, but TIMEOUT
did not. At probe uptime 1328.799 s CAN was error-warning with TEC=0 and
REC=124; by 1329.899 s both counters were zero again. Thus a successful reply
or a later zero counter did not establish stable communication, and querying
other node IDs was not necessary to reproduce the anomaly. Further requests
were stopped. The user was asked to power down and temporarily isolate the
GM6020 CAN branch, retaining the original DM wiring, then report termination
configuration or a powered-off H-to-L resistance measurement before retrying.

Termination change and communication recovery
=============================================

The user measured an open circuit between H and L with the original wiring,
then enabled the GM6020's configurable 120-ohm terminator. The reported bus
resistance became 120 ohms; a second end terminator was not available. This
single-terminator configuration was retained for the following bench tests.
Fourteen initial register reads and a further 100 consecutive reads all received
matching responses. Observed TEC/REC and queue-drop counts stayed zero while
GM6020 feedback continued. This supports missing termination as a contributor
to the earlier errors; it is not qualification of arbitrary wiring lengths or
an assertion that one terminator replaces proper termination at both ends.

Bounded motor and controller tests
==================================

The temporary application attached both DT devices to one production GZRL bus
coordinator on CAN1. GM6020 ID 2 used its usual grouped 0x1ff command slot;
DM node 2 used control ID 0x002 and feedback ID 0x005. Both devices used the
production protocol, lifecycle, queue and asynchronous transmission paths.
Only the application supplied test targets, timing and additional guards.

DM PMAX/VMAX/TMAX were populated from the readback, not generic DM4310 defaults.
The test used gear_ratio=1 to work directly in protocol-native angular units;
the motor's own Gr=10 was not changed or applied a second time. Reported
positions/speeds are feedback-coordinate observations, not independently
calibrated rotor/output-shaft measurements. Geared-variant axis interpretation
must be established before reusing this fixture in a robot's kinematics.

TIMEOUT was temporarily written as 2000, corresponding to 100 ms at the manual's
50-us counting period, and read back twice. It was not saved to flash. The
application additionally enforced a 20-ms command timeout, 50-ms feedback and
TX timeouts, and initially a two-second run deadline. Startup required a new
explicit Shell request. Tests used the following sequence:

* Zero effort: enable confirmed with drive state 1; deadline stop confirmed
  with state 0. GM6020 remained stopped during DM-only tests.
* Direct torque: +/-0.02 N m and +0.1 N m produced signed torque feedback with
  negligible movement. +0.2 N m moved approximately 0.003 rad. -0.2 N m reached
  approximately -1.501 rad/s and activated the initial 1.5-rad/s test guard;
  disable was confirmed. This was a test threshold, not a CAN or drive fault.
  Fresh disabled-state polling subsequently confirmed stationary position.
* At the user's confirmation that higher speed was acceptable, subsequent
  short tests retained the torque limits and used a 10-rad/s observation guard.
* Software speed PI: Kp=0.2, Ki=1.5, Kd=0, output limit +/-0.5 N m and integral
  limit +/-0.3 N m. A -0.1 rad/s target moved approximately -0.038 rad in the
  two-second trial; sampled speed reached -0.124 rad/s and absolute feedback
  torque reached approximately 0.241 N m. This verifies controller composition,
  not a completed speed-accuracy specification.
* Software position composition: position P gain 2 produced a speed target
  limited to +/-0.1 rad/s for the same PI. A +0.05 rad displacement request moved
  approximately +0.056 rad, with sampled speed up to 0.505 rad/s. No general
  settling-time, loaded-accuracy or repeatability claim is made.
* Full MIT command: position=current+0.05 rad, velocity=0.05 rad/s, Kp=0.25,
  Kd=0.01 and torque feedforward=0.2 N m exercised all five fields. Movement was
  approximately +0.140 rad; this was command-path validation, not precision
  positioning. The feedforward limit alone does not bound MIT total output:
  using the entire +/-pi position and +/-30 rad/s velocity mapping ranges,
  these gains/targets bound the nominal combined command below 2.1 N m, below
  20% of the manual's 12.5-N m peak torque. Observed sampled feedback torque
  peaked near 0.202 N m.
* Two-second simultaneous control: GM6020 raw command 800 and DM speed PI
  moved approximately +0.34 rad and -0.04 rad respectively, then both stopped.
  Both remained RUNNING during the commanded interval with no CAN error,
  queue drop, failed transmission or stalled transmission.
* Deliberately withholding periodic commands after startup triggered the
  software command-expiry fault approximately 21 ms after the start request
  (35.823 to 35.844 s). Disable was confirmed. A stopped-state clear-error
  operation was accepted and received disabled feedback; no physical drive
  fault was deliberately induced. Explicit zero-target restart cleared the
  software fault and ran normally before stopping. Clear-error did not itself
  restore running permission.

DM does not continuously report while idle. Stopped-state checks used explicit
0xfd disable requests to obtain fresh feedback, outside active motion. A later
online=false snapshot without polling is expected and is not evidence of a
new bus failure. Near-zero reported velocities of -0.007 or -0.021 rad/s were
interpreted together with stable position and drive state 0, not as exact zero
from this quantized feedback.

Twenty-second simultaneous rotation
====================================

At the user's explicit request, a further run used GM6020 raw command 1500 and
DM torque feedforward 0.5 N m, with Kp=Kd=0. The firmware command limits were
1500 raw counts and 0.5 N m respectively. The run was explicitly started,
lasted exactly 20 seconds (5.473--25.473 s), and ended automatically. A 29-rad/s
feedback guard and a 60-degree-C temperature guard remained active; position
travel was not limited for this continuous-rotation test. Neither guard fired.

.. list-table::
   :header-rows: 1

   * - Observation
     - GM6020
     - DM-J4310-2EC
   * - Feedback frames counted during the active interval
     - 20013
     - 20000
   * - Peak feedback speed observed by the control thread
     - 1.675 rad/s
     - 18.219 rad/s
   * - Maximum reported temperature
     - 27 degrees C
     - 28 degrees C
   * - Maximum observed newest-feedback timestamp step
     - 2000 us
     - 1200 us
   * - Frames coalesced between application snapshots
     - 355
     - 0

Both motors continuously rotated. DM accelerated toward approximately 18 rad/s;
GM6020 speed varied with constant voltage command. This was sustained rotation,
not constant-speed closed-loop regulation. Peak DM feedback torque was
approximately 0.529 N m, below the 2.5-N m user ceiling. GM6020 effort feedback
is in native counts and must not be interpreted as its voltage command or N m.

All 40001 coordinator submissions completed after stop, with zero RX queue
drops, send failures and TX stalls. CAN TEC/REC stayed zero in the per-cycle
checks; no runtime fault occurred. Control periods were 900--1100 us. The
355 coalesced GM frames were counted and decoded before the next application
snapshot, not discarded. Timestamp-step extrema describe the thread's latest
snapshot observations, not a hardware trace of every adjacent on-wire frame.
Neither motor protocol provides a sequence number proving absence of every
possible on-wire loss; the counters and expected feedback rates show **no
observed frame loss** during this run.

After stopping, repeated fresh disable polls confirmed DM drive state 0,
stable position and near-zero quantized speed; GM6020 reported zero speed.
The host summary parser initially confused a new SUMMARY line with a motor
snapshot after the run had finished. The complete saved UART transcript was
parsed with the correct snapshot filter; this did not interrupt or repeat the
physical run.

Protocol correction, checks and remaining scope
===============================================

The supplied V1.4 manual additionally defines fault states 3, 4 and 5 for the
dual-encoder variant. A new protocol test first failed because the existing MIT
decoder rejected those frames. The smallest fix accepts these states as faults;
reserved states 2, 6, 7 and 15 remain rejected without state mutation. The
existing lifecycle consumes these accepted faults through its normal fault path.
No sensor or calibration fault was physically induced to test this change.

The complete native_sim suite passed 56/56 cases across four configurations
with no warnings, including the new regression. The Type C test and final
sample builds passed. Focused clang-tidy on the changed production decoder,
formatting, REUSE and whitespace checks passed. Upstream Zephyr stayed clean.

Hardware zero was not changed, and its encoding remains covered by software
tests rather than this hardware session. Physical cable disconnection and
actual drive-watchdog stop timing, calibrated shaft units, loaded performance,
precision tuning and all DM family variants remain separate acceptance work.
The original TIMEOUT=0 and IDs 2/5 were restored and read back after testing;
no motor parameters were saved to flash. This does not leave a verified drive
watchdog configured for a future application.

Temporary firmware, host scripts, extracted manual pages, logs and test-build
outputs were removed after their necessary results were recorded here. The
production changes retained are the small MIT fault-state correction, its
regression test and protocol/validation documentation. The final board image
is again the stopped GM6020 observation sample with the conservative +/-800
limit, not an automatically running dual-motor test. It includes the decoder
fix, is 81900 bytes, and has SHA-256
``82006b1ef9c0849631faa44f68d3cef1621f80240334a4c14c45c7d25aa224ff``.
