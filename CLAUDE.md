<!--
SPDX-FileCopyrightText: 2026 GMaster contributors
SPDX-License-Identifier: Apache-2.0
-->

# GZRL project guidance

## Identity and baseline

- **GZRL** means GMaster Zephyr Robot Library, a new independent successor to
  the legacy STM32 HAL + FreeRTOS GSRL project.
- GZRL is an independent Git repository, the west manifest repository, and a
  Zephyr external module. Do not modify, vendor, or nest the sibling upstream
  `zephyr/` repository.
- The authoritative workspace manifest is `west.yml`. It selects the initial
  supported Zephyr baseline, **v4.4.1**, and imports that release's dependency
  manifest.
- The module integration entry points are `zephyr/module.yml`,
  `zephyr/CMakeLists.txt`, and `zephyr/Kconfig`. In a GZRL-managed workspace,
  west discovers the manifest repository as a module automatically.
- Use `EXTRA_ZEPHYR_MODULES` only when intentionally testing GZRL from an
  unrelated west workspace. Do not edit the imported upstream `zephyr/west.yml`
  to add GZRL.

## Architecture boundaries

GZRL is layered deliberately:

1. **Portable core** — algorithms, protocol codecs, data models, and
   deterministic state machines.
2. **Zephyr adaptation** — device integration, transport, scheduling, logging,
   settings, supervision, and inter-component messaging.
3. **Applications** — complete robot assemblies, board-specific topology,
   control composition, user interaction, and deployment configuration.

The portable core must not include Zephyr headers or depend on Devicetree,
zbus, Zephyr kernel primitives, LVGL, STM32 HAL, FreeRTOS, direct hardware I/O,
or implicit system time. Inputs, timestamps, and time steps are passed in
explicitly. This preserves a future Linux adaptation path.

Use Zephyr-native facilities in the adaptation layer rather than rebuilding
them: device model, Devicetree, Kconfig, CAN/UART/SPI/I2C/PWM/Counter APIs,
logging, shell, settings with NVS/ZMS, watchdog, zbus, and existing network or
USB protocol stacks. zbus is for cross-service state, commands, faults, and
telemetry; it is not the raw ISR or high-frequency control data plane. LVGL is
an optional presentation-edge dependency, never a portable-core dependency.

## Legacy GSRL migration policy

Use `../GSRL_refactor_original/` as a behavior, algorithm, and wire-protocol
reference only. Do not copy its CubeMX BSP, HAL handle APIs, global callback
managers, FreeRTOS/CMSIS glue, DWT/SysTick access, or legacy directory layout.
Migrate a capability by separating its portable algorithm/codec from the
Zephyr driver or service that uses it, then test both boundaries.

## Directory admission rules

- `samples/` contains focused, independently buildable demonstrations of one
  capability.
- `applications/` contains complete, independently buildable reference robots;
  do not add partial applications.
- Add `lib/` and `include/` only with the first real portable public API and
  implementation.
- Add `subsys/` only with a real Zephyr-bound robot service.
- Add `drivers/` and `dts/` only with a real device-model implementation and
  binding when upstream Zephyr has no suitable driver/API.
- Add `boards/<vendor>/<board>/` only with a complete, verified downstream
  Zephyr board port. At the same time, add `build.settings.board_root: .` and
  top-level `boards: [boards]` to `zephyr/module.yml`.
- For existing upstream boards, use application overlays and configuration
  fragments rather than copying a board port.
- Add `snippets/<name>/` only for a real reusable configuration profile; then
  add `build.settings.snippet_root: .` to `zephyr/module.yml`.
- Do not pre-create empty feature, UI/LVGL, Linux-port, board, driver, test, or
  subsystem directories.

## Agent orchestration

- Run no more than **5 subagents concurrently**. Before launching another
  subagent, wait for a running one to finish or stop it to keep token usage
  bounded.

## Engineering rules

- Prefer explicit byte-wise protocol encoding/decoding and Zephyr endian
  helpers. Do not map wire formats with packed structs, C/C++ bitfields,
  `reinterpret_cast`, or native floating-point memory layouts.
- Keep ISR callbacks minimal: capture, timestamp, enqueue, or signal only.
  Parse, log, run floating-point math, and execute control logic in thread or
  work context as appropriate.
- Keep real-time control state single-owner where possible. Use fixed storage
  and bounded queues in fast paths; avoid heap allocation, exceptions, RTTI,
  and dynamic containers there.
- PID and estimators receive explicit `dt`; health/offline state is timestamp
  based and getters have no side effects.
- New source files use SPDX Apache-2.0 headers unless a compatible imported
  license requires otherwise.

## Validation

After changing the manifest, module discovery, or architecture glue, at minimum
build and run `samples/smoke` on `native_sim` from the GZRL-managed workspace
without `EXTRA_ZEPHYR_MODULES`, and run Twister against the affected sample/test
tree. Keep the upstream Zephyr worktree clean.
