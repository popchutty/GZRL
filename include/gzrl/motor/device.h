/*
 * SPDX-FileCopyrightText: 2026 GMaster contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef GZRL_MOTOR_DEVICE_H_
#define GZRL_MOTOR_DEVICE_H_

#include <gzrl/motor/core.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Application-owned lifecycle; availability and running permission are distinct. */
enum gzrl_motor_run_state {
	GZRL_MOTOR_STOPPED,
	GZRL_MOTOR_STARTING,
	GZRL_MOTOR_RUNNING,
	GZRL_MOTOR_FAULT,
};

/** @brief Latched reasons for revoking running permission. */
enum gzrl_motor_fault {
	GZRL_MOTOR_FAULT_NONE,
	GZRL_MOTOR_FAULT_FEEDBACK,
	GZRL_MOTOR_FAULT_DRIVE,
	GZRL_MOTOR_FAULT_COMMAND,
	GZRL_MOTOR_FAULT_START,
	GZRL_MOTOR_FAULT_RX_LOSS,
	GZRL_MOTOR_FAULT_TX,
	GZRL_MOTOR_FAULT_BUS,
	GZRL_MOTOR_FAULT_TIME,
};

/** @brief Runtime policy, separate from immutable Devicetree topology. All fields required. */
struct gzrl_motor_policy {
	uint64_t command_timeout_us;
	uint64_t startup_timeout_us;
	uint64_t retry_interval_us;
	uint64_t tx_timeout_us;
	uint8_t max_attempts;
};

/** @brief Consistent snapshot read by the application's owning control thread. */
struct gzrl_motor_snapshot {
	struct gzrl_motor_feedback feedback;
	enum gzrl_motor_run_state state;
	enum gzrl_motor_fault fault;
	bool online;
	bool operation_pending;
	int last_error;
};

/** @brief Configure before bus attachment; topology fields must match the DT instance. */
int gzrl_motor_device_configure(const struct device *dev, const struct gzrl_motor_config *config,
				const struct gzrl_motor_policy *policy);
/** @brief Read immutable model, node ID, feedback ID and native CAN controller. */
int gzrl_motor_device_topology(const struct device *dev, struct gzrl_motor_config *topology,
			       const struct device **can);
/** @brief Query supported command bits, or zero for an invalid device. */
uint32_t gzrl_motor_device_capabilities(const struct device *dev);
/** @brief Explicitly authorize startup; does not transmit until bus_flush(). */
int gzrl_motor_device_start(const struct device *dev, uint64_t now_us);
/** @brief Revoke permission and schedule safe output, without implicitly engaging a brake. */
int gzrl_motor_device_stop(const struct device *dev);
/** @brief Replace the latest motion command; invalid commands revoke active permission. */
int gzrl_motor_device_command(const struct device *dev, const struct gzrl_motor_command *command,
			      uint64_t now_us);
/**
 * @brief Request a bounded, one-shot status/clear-error/set-zero/brake operation while stopped.
 *
 * Enable/disable must use start/stop. Brake release is only performed by authorized startup.
 * Successful CAN completion is not a promise that a drive applied a maintenance operation.
 */
int gzrl_motor_device_operation(const struct device *dev, const struct gzrl_motor_command *command,
				uint64_t now_us);
/** @brief Reset software displacement origin while stopped and feedback is fresh. */
int gzrl_motor_device_reset_position(const struct device *dev, uint64_t now_us);
/** @brief Read status without advancing time, fault counters, or protocol state. */
int gzrl_motor_device_snapshot(const struct device *dev, uint64_t now_us,
			       struct gzrl_motor_snapshot *snapshot);

#ifdef __cplusplus
}
#endif
#endif /* GZRL_MOTOR_DEVICE_H_ */
