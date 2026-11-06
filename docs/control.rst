.. SPDX-FileCopyrightText: 2026 GMaster contributors
.. SPDX-License-Identifier: Apache-2.0

Portable control algorithms
###########################

The ``gzrl/control.h`` interface contains independent C control primitives. It
has no dependency on Zephyr, motor protocols, clocks, dynamic allocation, or a
particular feedback source. The application owns each object and determines
when it advances. Zephyr applications select ``CONFIG_GZRL_CONTROL``; C++
applications use the same C interface.

PID time and limits
*******************

``gzrl_pid_update()`` accepts a target, measurement, elapsed time in seconds,
and an output pointer. The caller supplies positive ``dt_min`` and ``dt_max``
limits. A nonfinite input, invalid time step, or unrepresentable intermediate
returns an error without changing history or output. An application must treat
such an error as a failed control update and stop or invalidate its actuator
command; it must not repeatedly send an old command.

The gain convention is:

.. math::

   P_k = K_p e_k,\qquad \Delta I_k = K_i e_k\Delta t_k

Differentiation uses the actual interval between updates. By default:

.. math::

   D_k = -K_d(y_k-y_{k-1})/\Delta t_k

``GZRL_PID_D_ERROR`` instead differentiates the error. The first accepted update
after initialization or reset sets the derivative contribution to zero.

In positional mode the proposed output is ``P + I_candidate + D``. In
incremental mode it is ``previous_output + (P - previous_P) +
(I_candidate - previous_I) + (D - previous_D)``. Both modes limit the proposed
integral to the configured integral interval. If its change would drive the
proposed output further beyond an output limit, the new integral is not
committed. The output is still the saturated proposed output, allowing even a
large first pure-integral step to reach the limit. An integral change in the
opposite direction is allowed to unwind the controller.

Position and incremental modes agree before saturation, subject to floating
point rounding. After saturation their histories can differ: incremental mode
accumulates from the actual saturated output. Diagnostic ``p``, ``i`` and ``d``
therefore do not always sum to ``output``. Integral limits apply to the
integral contribution in output units. Both limit intervals must contain zero;
reset clears history and produces no nonzero output. Gains are nonnegative;
sensor and actuator polarity belongs to the application's coordinate mapping.

Parameter replacement uses ``gzrl_pid_init()`` and resets history. There is no
implicit live retuning or bumpless mode switch. Stop the actuator, reset the
controller, establish a valid feedback sample and explicitly restart when
changing modes or recovering from a fault.

Migration of old PID gains
**************************

The old GSRL implementation accumulated and differentiated once per call,
without an explicit time step. If its fixed sample interval was ``T`` seconds,
the corresponding time-based gains are:

.. code-block:: text

   kp_new = kp_old
   ki_new = ki_old / T
   kd_new = kd_old * T

These relationships also apply to the unsaturated incremental recurrence.
Matching the old error derivative additionally requires
``GZRL_PID_D_ERROR``. Startup derivative handling, conditional integration,
time jitter and saturation behavior deliberately differ, so these conversions
are initial estimates, not permission to reuse old gains without validation.

Composition and cascade scheduling
**********************************

The data path is explicit:

.. code-block:: text

   feedback -> optional filter -> controller -> bounded actuator command

A speed loop uses velocity as its measurement; a multi-turn position loop uses
valid unwrapped position; a current loop uses current only when its units and
validity are established by the device. External sensors work identically.
No motor object stores a controller pointer.

``gzrl_cascade_update()`` contains two independent PIDs. The application passes
``outer_due = true`` only when the outer loop is scheduled, with the actual
elapsed outer interval. Every accepted call advances the inner loop using its
own elapsed interval. Between outer updates the last outer result is held.
The first update must run the outer loop. If either loop fails, neither loop
advances. Limits on the outer result are in the inner loop's reference units,
for example rad/s for an outer position loop and inner speed loop.

``gzrl_angle_error()`` supplies the shortest signed target-minus-measurement
error. A loop with error differentiation can pass that error as setpoint and
zero as measurement. A loop with measurement differentiation needs continuous
unwrapped feedback; construct a local target as ``measurement + error``.
Feeding wrapped measurements directly to a differentiator creates a false
velocity impulse at the wrap. A target exactly opposite the current angle has
an unavoidable direction choice, defined here as the negative half-turn.

Independent filters
*******************

The first-order low-pass filter uses ``alpha = dt / (time_constant + dt)`` and
seeds its state from the first valid sample after reset. It does not introduce
an artificial zero-valued startup sample.

Moving-average and median windows use caller-owned arrays. During startup they
operate on the available samples. Median filtering uses a separate,
non-overlapping scratch array; even-sized windows average the two middle
values. Insertion sorting bounds work by the selected window size, with
quadratic worst-case cost. Choose a small window and include that cost in the
application's control budget. Filter reset empties the window without reading
uninitialized storage.

Objects, their backing arrays, and output arguments must not overlap, except
where a function's declared output is explicitly a separate field of a
composite object used internally. Access each object from one thread or protect
it externally. Error returns never silently clamp invalid inputs.

Validation
**********

The ``gzrl.control`` Ztest scenario checks independent numerical sequences for
both PID forms, variable periods, both derivative policies, saturation and
unwinding, integral limits, transactional failures, multirate cascades,
first-order plant convergence, filter windows, angle wrapping and C++ linkage.

From the activated workspace:

.. code-block:: console

   west twister -T GZRL/tests/control -p native_sim -O GZRL/twister-out-control

The production files also compile as ordinary C17 without Zephyr include paths.
This check establishes the portable boundary; hardware tuning and actuator
validation are separate from the numerical tests.
