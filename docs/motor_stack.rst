.. SPDX-FileCopyrightText: 2026 GMaster contributors
.. SPDX-License-Identifier: Apache-2.0

Motor stack
###########

Architecture and ownership
**************************

The portable control library and motor protocol library have no dependency on
each other, Zephyr, hardware access, or implicit time. Applications compose
feedback, filters, controllers and native motor commands through ordinary data.
See ``control.rst`` for PID equations and ``motor_protocols.rst`` for model
profiles, native units, evidence and known firmware constraints.

The Zephyr layer provides ``gmaster,can-motor`` devices and one caller-owned
``gzrl_motor_bus`` per physical CAN controller. Device initialization only
checks dependencies. It never enables a drive, releases a brake or transmits.
All enabled GZRL endpoints on a controller must be attached together, so that
two callers cannot independently overwrite slots in a shared command frame.

One application thread owns configuration, lifecycle, command submission,
snapshot reads and bus processing. Other threads, including Shell, must pass
requests to that thread using bounded Zephyr queues. Do not invoke these APIs
from an ISR. No motor worker thread or high-rate zbus data path is introduced.

The callbacks only timestamp/copy/enqueue receive frames and publish transmit
completion using atomics. Their storage must remain alive until CAN has stopped
and ``gzrl_motor_bus_detach()`` succeeds. An attached bus must not be copied.
Initialization failure removes already-installed filters and leaves devices
unbound. CAN mode, bitrate, start, stop and recovery remain application-owned;
the library does not reset a potentially shared bus behind another user's back.

Build options and topology
**************************

``CONFIG_GZRL_CONTROL`` selects PID/cascade/filter primitives.
``CONFIG_GZRL_MOTOR_CORE`` selects portable codecs and feedback tracking.
``CONFIG_GZRL_MOTOR`` selects Zephyr device and bus integration and depends on
CAN. The latter selects the protocol core but does not select a controller
algorithm. All public headers support C++ linkage without requiring C++ in
production libraries.

Application overlays declare each endpoint's ``can`` phandle, ``model``,
``motor-id`` and, for MIT, ``feedback-id``. Motor topology does not belong in
the common board DTS. The application supplies limits, firmware scaling,
gear ratio, encoder offset and lifecycle timeouts through runtime configuration
before attachment. The topology fields in that configuration must match DT.
Firmware ranges are never inferred merely from a model's name.

The configurable maximum devices and receive depth bound storage and work.
The default capacities are 16 devices and 64 receive events per bus. Applications
must budget CAN bandwidth including feedback, arbitration and bit stuffing;
a 1 ms example control loop is not a promise that every model or bus population
can sustain 1 kHz transmission. ``process()`` accepts an explicit receive budget.

Application cycle
*****************

#. Read ``gzrl_motor_bus_now_us()``; it uses Zephyr's monotonic tick time, converted
   to microseconds. Resolution remains the configured tick resolution.
#. Process a bounded number of RX events and TX completions with
   ``gzrl_motor_bus_process()``. Decode uses reception time, not drain time.
#. Inspect feedback validity, online state and running permission. Reset the
   application controller on every start/stop/mode/fault transition.
#. Update the selected controller with the actual elapsed ``dt``. Wrapped-angle
   error and external-sensor feedback are application composition choices.
#. Submit a validated native motion command with ``gzrl_motor_device_command()``.
#. Flush the latest targets with ``gzrl_motor_bus_flush()``. Inspect its return
   code and diagnostic counters; queue acceptance is not transmission success.

Do not repeatedly submit the same stale user target merely to extend its life.
A running application controller needs an independent lease for its upstream
target. The sample demonstrates this distinction. A stalled application cannot
execute a software timeout; configure the drive's hardware communication watchdog
and arrange application supervision as part of the physical integration.

Scheduling and failures
***********************

Hardware receive filters are deduplicated by arbitration ID. A shared MIT
feedback ID is allowed only when the payload identifies distinct supported
nodes. Conflicting command IDs/slots and cross-endpoint command/feedback aliases
are rejected. RTR, extended and FD frames are rejected before decoding.

Receive overflow is counted, invalidates accumulated position and revokes motion
permission. Malformed packets do not refresh a device's online timestamp. Brake
acknowledgements do not refresh motion feedback. Recovery never silently restores
multi-turn position: establish a new reference explicitly when fresh feedback
is available and the device is stopped.

The bus coordinator constructs complete grouped frames; inactive slots are zero.
It uses round-robin TX-ID scheduling, one in-flight frame per ID and the latest
unsent target. It calls ``can_send()`` with a completion callback and
``K_NO_WAIT``. This avoids waiting for a TX mailbox, but does not claim that
upstream driver mutexes or on-wire transmission are time-bounded. The native
CAN API has no portable single-frame cancellation operation.

A completion callback may run before ``can_send()`` returns. Stable slot storage
and atomic completion publication handle that case. Generation numbers prevent
an old operation completion from satisfying a newer stop or maintenance request.
Mailbox contention is retried on later cycles with fairness; asynchronous errors
and excessive in-flight duration revoke permission. The application must handle
a stalled controller and stop/recover it as appropriate for other bus users.

Lifecycle and maintenance
*************************

``STOPPED -> STARTING -> RUNNING`` requires a new explicit request and fresh
feedback. MIT startup sends enable; MG startup releases the brake and requests
status. Both are bounded by attempt count, retry interval and startup deadline.
Group-protocol drives establish readiness from their periodic feedback.

``FAULT`` latches the reason and invalidates targets. A recovered drive remains
stopped until the application requests startup again. No implicit clearing of
over-current, over-temperature or other physical faults is performed. An explicit
maintenance operation can request the protocol's clear-error command while stopped.

Stop revokes permission immediately in software, and schedules MIT disable or
zero effort for the other profiles. It does not engage a mechanical brake.
Already-in-flight frames cannot be retracted by this layer; shutdown must preserve
the CAN link long enough to transmit and verify the safe state before stopping
the controller. Explicit brake engagement is available while stopped. Brake
release is part of an authorized startup, not a free-standing maintenance action.

Maintenance operations have a deadline and bounded attempts. Their completion
indicates successful CAN transmission, not a fabricated acknowledgement from a
drive that does not provide one. Hardware-zero commands invalidate software
position continuity; software origin reset is a separate operation.

Legacy capability mapping
*************************

* HAL CAN initialization/callback managers: native Zephyr CAN controller plus
  bounded timestamped receive queue and asynchronous transmit coordination.
* ``getMotorControlData``: explicit command submission and flush; snapshots and
  getters do not advance offline counters or mutate lifecycle.
* ``getMergedControlData``: complete per-ID group assembly across all registered
  slots, with duplicate/conflicting ownership rejected.
* Open-loop/native effort: typed command with explicit raw or N m units and limits.
* Software angle, speed, revolution, current and external-feedback loops:
  independent PID/cascade and application-selected feedback composition.
* Controller polarity: an explicit application coordinate transform applied to
  feedback and target/output, not a hidden motor-class flag.
* Encoder offset, angle, speed and turns: native-angle and output-displacement
  feedback plus explicit software reference reset; turns are position/(2*pi).
* Automatic drive enable/brake release: an authorized bounded startup state machine.
* Automatic error clearing after reconnect: explicit stopped maintenance and a
  new startup request, preserving capability without restoring an old target.
* MG internal speed/angle/multi-turn loops and brake: capability-checked commands.

Validation record
*****************

See ``motor_validation.rst`` for dated software results and the pending physical
acceptance matrix.

Automated tests cover core numeric/protocol contracts, independent C/C++ linkage,
CAN fault injection and native loopback. CI builds the Type C motor sample and
runs the host tests, REUSE, format checks and focused clang-tidy over every new
production source. Host standalone compilation checks that the core does not
accidentally acquire a Zephyr dependency.

Physical motor validation is **pending** for every profile until a dated entry
records the actual motor, firmware, configuration, wiring, board, limits, logs
and observed result. A passing build or simulated CAN exchange is not a physical
pass. After the user connects selected motors and confirms the environment is
ready, run observation first, then explicit startup/stop, bounded direct output,
software/drive-side loops as supported, disconnect, and deliberate restart.
Record untested capabilities as NOT TESTED, not SUCCESS.

The software tests and sample are reproducible with the workspace environment
activated:

.. code-block:: console

   west twister -T GZRL/tests -p native_sim -O GZRL/twister-out-tests
   west build -p always -b robomaster_type_c \
     -d GZRL/build/motor-control GZRL/samples/motor_control

The normative transport behavior is documented in the
`Zephyr CAN controller API <https://docs.zephyrproject.org/latest/hardware/peripherals/can/controller.html>`_.
The actual integration is compiled against the workspace's pinned Zephyr v4.4.1.
