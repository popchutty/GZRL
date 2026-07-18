<!--
SPDX-FileCopyrightText: 2026 GMaster contributors
SPDX-License-Identifier: Apache-2.0
-->

# GZRL

GZRL (GMaster Zephyr Robot Library) is a new robot-control library built as a
[Zephyr external module](https://docs.zephyrproject.org/latest/develop/modules.html).
It is the planned successor to the legacy STM32 HAL + FreeRTOS based GSRL
project, but it is a new codebase and does not preserve the legacy API.

## Status

The repository provides the external-module integration skeleton, a smoke
sample, and a downstream Zephyr board port for the DJI RoboMaster Development
Board Type C. No legacy GSRL algorithms, protocols, robot services, or complete
applications have been migrated yet.

The Type C port is build-validated against Zephyr v4.4.1. Physical peripheral
validation is still tracked separately and must be completed on real hardware.

The initial Zephyr compatibility baseline is **v4.4.1**. The repository's
[`west.yml`](west.yml) is the authoritative workspace manifest: it selects that
Zephyr release and imports the dependency revisions maintained by Zephyr.

## Create a workspace

GZRL uses Zephyr's T2 west topology: this repository is both the manifest
repository and a Zephyr module. To create a workspace, set
`GZRL_REPOSITORY_URL` to the repository's published URL, then run:

```sh
mkdir gzrl-workspace
cd gzrl-workspace
git clone "$GZRL_REPOSITORY_URL" GZRL
west init -l GZRL
west update
```

In a workspace initialized this way, west discovers GZRL through
`GZRL/zephyr/module.yml`.

## Build the smoke sample

From the workspace root with its environment activated:

```sh
west build -p always \
  -b native_sim \
  -d GZRL/build/smoke \
  GZRL/samples/smoke
west build -d GZRL/build/smoke -t run
```

Expected output includes:

```text
GZRL module smoke sample
```

## Build for the RoboMaster Type C

The downstream `robomaster_type_c` board is discovered from this module without
setting `BOARD_ROOT` or `EXTRA_ZEPHYR_MODULES`:

```sh
west build -p always \
  -b robomaster_type_c \
  -d GZRL/build/robomaster-type-c \
  zephyr/samples/hello_world
```

USB CDC ACM is the default console. OpenOCD with a CMSIS-DAP/DAP-Link probe is
the default flash/debug runner; STM32CubeProgrammer, ST-LINK GDB server, J-Link,
and STM32 ROM DFU are also registered. See the board documentation under
`boards/dji/robomaster_type_c/doc/` for wiring, supported peripherals, and
validation status.

## Repository scope

- `zephyr/` contains the external-module metadata and Zephyr integration entry
  points.
- `boards/` contains complete downstream Zephyr board ports maintained by GZRL.
- `samples/` contains small, focused, independently buildable examples.
- `applications/` is reserved for complete reference robot applications such as
  chassis or gimbal configurations.

Future portable algorithms, codecs, data models, and deterministic state
machines will be introduced separately from Zephyr-bound drivers and services,
so that a Linux adaptation remains feasible.

See [`CLAUDE.md`](CLAUDE.md) for the project architecture and development
constraints.

## License

Copyright 2026 GMaster contributors.

Licensed under the [Apache License, Version 2.0](LICENSE).
