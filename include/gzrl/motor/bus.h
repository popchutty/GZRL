/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef GZRL_MOTOR_BUS_H_
#define GZRL_MOTOR_BUS_H_

#include <gzrl/motor/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#ifdef __cplusplus
extern "C" {
#endif

struct gzrl_motor_bus;

/** @brief Fixed receive event storage; timestamp belongs to reception, not queue draining. */
struct gzrl_motor_rx_event {
	struct gzrl_motor_frame frame;
	uint64_t timestamp_us;
	uint32_t serial;
};

/** @brief Stable callback storage. Applications must not access these implementation fields. */
struct gzrl_motor_tx_slot {
	struct gzrl_motor_bus *bus;
	struct can_frame frame;
	atomic_t completion;
	atomic_t error;
	uint64_t submitted_us;
	uint32_t members;
	uint32_t generations[CONFIG_GZRL_MOTOR_MAX_DEVICES];
	uint8_t actions[CONFIG_GZRL_MOTOR_MAX_DEVICES];
	bool stalled;
};

/** @brief Diagnostic totals, updated by the control thread while draining callback events. */
struct gzrl_motor_bus_stats {
	uint32_t received;
	uint32_t rejected;
	uint32_t dropped;
	uint32_t submitted;
	uint32_t completed;
	uint32_t failed;
	uint32_t mailbox_busy;
	uint32_t stalled;
};

/**
 * @brief Caller-owned static bus storage. Zero initialize before first attach.
 *
 * All public APIs require one owning application thread. CAN callbacks only touch queue
 * and atomic fields. Keep this storage alive until the controller is stopped and detach
 * succeeds. Do not copy attached instances. No private worker thread is created.
 */
struct gzrl_motor_bus {
	const struct device *can;
	const struct device *devices[CONFIG_GZRL_MOTOR_MAX_DEVICES];
	struct gzrl_motor_tx_slot slots[CONFIG_GZRL_MOTOR_MAX_DEVICES];
	int filters[CONFIG_GZRL_MOTOR_MAX_DEVICES];
	struct k_msgq rx_queue;
	char rx_storage[CONFIG_GZRL_MOTOR_RX_DEPTH * sizeof(struct gzrl_motor_rx_event)];
	atomic_t dropped;
	atomic_t rx_serial;
	atomic_t accepting;
	struct gzrl_motor_bus_stats stats;
	size_t count;
	size_t slot_count;
	size_t filter_count;
	size_t next_slot;
	uint64_t last_process_us;
	bool time_valid;
	bool attached;
};

/** @brief Monotonic Zephyr time in microseconds; use the same clock for all adapter calls. */
uint64_t gzrl_motor_bus_now_us(void);
/**
 * @brief Bind configured DT motor devices and register deduplicated receive filters.
 *
 * Call with CAN stopped. Every enabled GZRL motor on this controller must be included;
 * this prevents partial group ownership. One coordinator owns all GZRL motor IDs on a bus.
 * The application retains ownership of CAN mode/bitrate/start/stop/recovery operations.
 */
int gzrl_motor_bus_attach(struct gzrl_motor_bus *bus, const struct device *can,
			  const struct device *const *devices, size_t count);
/** @brief Drain at most rx_budget frames and advance lifecycle. Returns first error or zero. */
int gzrl_motor_bus_process(struct gzrl_motor_bus *bus, uint64_t now_us, size_t rx_budget);
/** @brief Commit newest targets in round-robin ID order; never wait for a TX mailbox. */
int gzrl_motor_bus_flush(struct gzrl_motor_bus *bus, uint64_t now_us);
/**
 * @brief Remove filters and bindings after application stops CAN; returns -EBUSY if unsafe.
 *
 * A successful detach discards pending safe-output work because the controller is stopped.
 * The application must ensure the drives have stopped before shutting down communication.
 */
int gzrl_motor_bus_detach(struct gzrl_motor_bus *bus);

#ifdef __cplusplus
}
#endif
#endif /* GZRL_MOTOR_BUS_H_ */
