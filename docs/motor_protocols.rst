.. SPDX-FileCopyrightText: 2026 GMaster contributors
.. SPDX-License-Identifier: Apache-2.0

Motor protocol core
###################

Contract and ownership
**********************

``<gzrl/motor/core.h>`` is a C API with C++ linkage guards. The implementation
uses only standard C headers. It contains no Zephyr, HAL, RTOS, device, clock,
allocation, or PID dependency. The caller owns every ``gzrl_motor`` and
serializes its operations. Configuration is immutable after successful
``gzrl_motor_init()``; reinitialization discards feedback and the position
origin and must only be done while the corresponding device is stopped.

The core converts bytes, maintains feedback, and produces commands. It does
not send frames or automatically enable, clear, release a brake, or restart
a motor. Those operations are explicit commands handled by the Zephyr runtime.

Every protocol accepts exactly eight-byte, standard-identifier classic CAN
DATA frames. The portable frame flags must be zero. Extended, remote, FD,
incorrect-length, incorrectly addressed, and unsupported packets do not update
feedback. Encoding checks capability, units, finite numbers, and field limits.
On failure it clears the output frame, including its length, rather than
leaving an earlier command available for accidental transmission.

Supported profiles
******************

.. list-table::
   :header-rows: 1
   :widths: 15 13 22 22 28

   * - Profile
     - Node IDs
     - Command CAN ID
     - Feedback CAN ID
     - Command units and hard envelope
   * - GM6020
     - 1--7
     - 0x1ff / 0x2ff
     - 0x204 + ID
     - Raw voltage-command counts, +/-30000
   * - M3508 / C620
     - 1--8
     - 0x200 / 0x1ff
     - 0x200 + ID
     - Raw current-command counts, +/-16384
   * - M2006 / C610
     - 1--8
     - 0x200 / 0x1ff
     - 0x200 + ID
     - Raw current-command counts, +/-10000
   * - DM4310 / DM2325 MIT
     - 1--15
     - Node ID
     - Explicit ``master_id`` (0--0x7ff)
     - N m, limited by configured TMAX
   * - Damiao one-to-four firmware
     - 1--8
     - 0x3fe / 0x4fe
     - 0x300 + ID
     - Native raw command counts, signed 16-bit envelope
   * - LKMG legacy 16-bit encoder profile
     - 1--32
     - 0x140 + ID
     - 0x140 + ID
     - Raw torque-current command counts, +/-2048

``effort_limit`` is required and may narrow the hard envelope. A protocol
envelope is not a recommended operating current or mechanical limit.
M3508 and M2006 are separate profiles despite sharing framing, because their
drive limits and feedback validity differ. C610 feedback bytes 6--7 are
reserved: M2006 never advertises a temperature measurement.

The original GM6020 voltage-command protocol is the supported variant; a new
firmware's different command modes are not silently treated as this profile.
LKMG here means the legacy firmware framing, eight-byte replies, and a
65536-count encoder cycle. Other encoder resolutions and protocol revisions
require their own verified profile, not a change to all LKMG devices.

The four-slot protocols use slot ``(node_id - 1) % 4`` and split IDs 1--4
and 5--8 across two control IDs. DJI control and feedback words are big endian.
Damiao one-to-four control words are little endian while its feedback words
are big endian. Its feedback speed is signed RPM times 100; conversion retains
the fractional RPM rather than first dividing into an integer.

``gzrl_motor_group_add()`` starts from a zero-initialized group, writes one
slot, and rejects a duplicate slot, different control ID, or different byte
ordering. It leaves unpopulated slots at zero and leaves an existing group
unchanged on failure. A bus coordinator must additionally detect conflicts
between command IDs, feedback IDs, and incompatible device profiles before
installing any receive filters.

DM MIT specifics
================

Position uses 16 bits; velocity, Kp, Kd, and torque use 12 bits. Integer
quantization uses the full ``2**bits - 1`` interval and truncates toward zero
after mapping to a nonnegative code. Zero is therefore between two adjacent
codes for symmetric ranges: tests must use the quantization tolerance rather
than require exact zero feedback.

The full MIT command accepts motor-native position in rad, velocity in rad/s,
Kp in [0, 500], Kd in [0, 5], and feed-forward torque in N m. The effort-only
command encodes zero position, velocity, Kp and Kd. The effort unit must be
``GZRL_MOTOR_EFFORT_NEWTON_METRE``; the default raw-count unit is rejected.

Special commands contain seven 0xff bytes followed by:

* 0xfb: clear error;
* 0xfc: enable;
* 0xfd: disable;
* 0xfe: set hardware zero.

A full MIT motion command whose quantized bytes collide with any of these
four reserved packets is rejected with ``-ERANGE``. It must never silently
turn a position/torque request into a lifecycle or hardware-zero operation.

Feedback byte 0 contains the four-bit motor ID and four-bit state. This
legacy variant supports node IDs 1--15, even though CAN identifiers have more
bits. Several motors may share a master ID only if the receiver demultiplexes
the embedded motor ID. States 0 and 1 mean disabled and enabled; 3--5 and 8--14
are reported faults. DM-J4310-2EC manual V1.4 defines 3--5 as output-axis
calibration, sensor output and motor-encoder calibration faults respectively.
Reserved states are rejected rather than interpreted as healthy.
MOS and coil temperature are separate valid fields, and torque is
kept as a floating-point value without the legacy int16 truncation.

P/V/T limits must match firmware configuration. In particular, PMAX is a
quantization scale and does not prove the physical wrap period. With
``position_wrap_rad == 0``, the core accumulates ordinary native-position
differences but invalidates continuity on a jump larger than PMAX. An
application may configure a positive wrap period only after confirming the
firmware behavior. The original PMAX=pi setup may explicitly select 2*pi
when that wrap behavior has been verified. The library does not guess it.

LKMG specifics
==============

The supported command set is:

* 0xa1: signed raw torque-current command;
* 0xa2: signed speed, little-endian int32 in 0.01 degree/s;
* 0xa6: single-turn motor-native angle in 0.01 degree, direction byte 0 for
  clockwise and 1 for counterclockwise, and uint16 speed limit in degree/s;
* 0xa4: signed multi-turn angle in 0.01 degree with the same speed-limit field;
* 0x9c: motion/status query;
* 0x8c: explicit brake command, byte 1 equal to 0 for engaged and 1 for released.

Single-turn angles belong to the native motor axis and lie in [0, 2*pi).
They are not multiplied by the external gearbox ratio. Speed commands,
multi-turn angle commands, and position-command speed limits are output-axis
quantities and are converted to native units using the configured ratio.
Wire integer overflow and a speed limit that would quantize below 1 degree/s
are rejected before any integer cast.

Motion replies for 0x9c, 0xa0--0xa8, and 0xad share the temperature/current/
speed/encoder fields. Temperature is signed; encoder position uses modulus
65536, not divisor 65535. A 0x8c acknowledgement updates brake validity and
status sequence but never refreshes the motion timestamp or position.

Units and position validity
***************************

The ``gear_ratio`` is the number of native-reported shaft revolutions per
application output revolution. It must be finite and positive. With firmware
that already applies its own gear ratio, this parameter describes only the
remaining external ratio; do not apply the built-in gearbox twice.

``native_position_rad`` preserves the offset-adjusted protocol position.
``rotor_angle_rad`` presents that position modulo 2*pi. ``velocity_rad_s`` and
the accumulated ``position_rad`` refer to the application output shaft.
Electrical current, raw drive counts, and native torque are not multiplied
by the ratio: output torque would additionally require mechanical information
which the protocol does not supply.

DJI and LKMG effort feedback is identified as native raw counts. Damiao
one-to-four reports mA under the legacy firmware contract; DM MIT reports
N m. These units must be checked before connecting an effort-feedback loop.
The source manuals document C620 command scaling (+/-16384 to +/-20 A) and
C610 command scaling (+/-10000 to +/-10 A), but the public command API keeps
raw counts explicit rather than conflating command scaling with feedback or
unknown firmware variants.

The first motion sample establishes zero accumulated displacement. Encoder
wrapping uses the shortest displacement and requires less than half a native
wrap period of motion between accepted samples. An exactly half-period jump
is ambiguous and invalidates accumulated position. Even without a timeout,
the application must choose feedback rates sufficient for its maximum speed.

A feedback gap at least ``feedback_timeout_us`` long or an explicit
``gzrl_motor_invalidate_position()`` clears accumulated-position validity.
Future frames continue refreshing angle, speed and drive state but do not
silently restore multi-turn validity. ``gzrl_motor_reset_position()`` is an
explicit software-origin reset; it does not send a hardware zero command.
The runtime must require a fresh sample before using that reset for motion.

Receive timestamps are monotonic microseconds supplied by the adapter at
reception. Backwards timestamps are rejected without mutation. Equal
timestamps are accepted for clocks with coarser resolution. A pure
``gzrl_motor_online()`` query checks the motion timestamp, including at time
zero, and does not advance state or count failed reads. Unavailable fields
are identified by the feedback validity mask, not by a fabricated zero.

Legacy functionality mapping
****************************

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Legacy behavior
     - Replacement
   * - HAL header and internal control-data getters
     - Explicit portable frame from ``gzrl_motor_encode()``; Zephyr adapter sends it
   * - ISR and queue decoding methods
     - One receive operation in owner-thread context, with captured receive time
   * - Pairwise control-data merging
     - Per-bus, per-ID frame assembly using explicit slot ownership
   * - Read-count-based connection state
     - Pure timestamp-based online query, independent of getter frequency
   * - Angle, velocity, revolutions, current and temperature getters
     - Typed feedback snapshot, validity mask and explicit units
   * - Encoder offset and revolutions reset
     - Initialization offset and explicit software-origin reset
   * - Open-loop output
     - Native-unit effort command with explicit application limit
   * - Angle/velocity/revolutions/current/external-feedback software loops
     - Independent controller and feedback composition in the application
   * - Controller replacement and output polarity
     - Application-owned algorithm state and explicit signal transformation
   * - Automatic DM clear/enable and MG brake release
     - Explicitly authorized runtime state transitions and bounded operations
   * - MG internal speed/single-turn/multi-turn loops
     - Separate capability-checked native command operations

References and validation status
********************************

The checked-in legacy ``dvc_motor.cpp`` establishes the migrated firmware
variants and behavior baseline. Independent manufacturer sources confirm the
DJI framing and limits and the LKMG command directions:

* `GM6020 manual, 2020 v1.2 <https://rm-static.djicdn.com/tem/17348/RoboMaster%20GM6020%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.pdf>`_
* `C620 manual v1.01 <https://cdn-hz.robomaster.com/robomasters/public/document/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.01.pdf>`_
* `C610 manufacturer manual <https://cdn-hz.robomaster.com/tem/RM%20C610%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%20%E5%8F%91%E5%B8%83%E7%89%88.pdf>`_
* `M2006 manufacturer demo protocol <https://cdn-hz.robomaster.com/tem/RoboMaster%20M2006%20%E5%8A%A8%E5%8A%9B%E7%B3%BB%E7%BB%9F%20DEMO%20%E7%A4%BA%E4%BE%8B%E7%A8%8B%E5%BA%8F.pdf>`_
* `LK manufacturer MF/MG protocol v2.35, mirrored copy <https://www.scribd.com/document/826568712/%E7%93%B4%E6%8E%A7MF9025>`_

The Damiao one-to-four profile currently follows the legacy documented
framing, including its mA feedback unit. It requires matching firmware
identification during hardware acceptance; this is not a claim that all
Damiao firmware uses this protocol.

``tests/motor_core`` exercises all profiles, command vectors, addressing,
group assembly, fractional feedback, native/output axis conversion, invalid
inputs, lifecycle command encoding, timestamps and position discontinuities.
These are software tests, not evidence of physical motor validation. All
profiles require hardware acceptance records identifying actual motor,
drive firmware, wiring, configured scaling, limits and measured behavior.
