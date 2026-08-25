/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Process Mission
 *
 * Read-only diagnostic snapshots exposed by the DDS and signal-chain
 * modules for the "dds" shell command set.
 */

#ifndef DDS_DIAG_H
#define DDS_DIAG_H

#include <zephyr/sys/util.h>

#include <stdbool.h>
#include <stdint.h>

/* One-line thread state strings. */
#define DDS_DIAG_THREAD_STATE_LEN 24

struct dds_diag {
	/* Session / entity state. */
	bool initialized;
	bool session_ready;
	uint8_t entity_ready_mask;	/* bit0 support .. bit5 executor subscription */
	uint32_t session_rebuilds;
	uint32_t publish_failures;
	uint32_t ping_failures_total;

	/* TX pending slot (latest value only). */
	uint32_t pending_tx_seq;
	bool tx_pending;

	/* RX side. */
	int32_t received_linux_seq;
	uint32_t recv_sem_count;	/* queued, not yet handled replies */

	/* Threads. */
	char supervisor_state[DDS_DIAG_THREAD_STATE_LEN];
	char reply_state[DDS_DIAG_THREAD_STATE_LEN];
	size_t supervisor_stack_free;
	size_t reply_stack_free;
};

struct signal_chain_diag {
	uint32_t timer_ticks;
	uint32_t control_count;
	uint32_t missed_ticks;
	uint32_t tx_seq;		/* zephyr heartbeat seq sent */
	uint32_t rx_count;		/* linux heartbeats received */
	uint32_t last_linux_seq;
	uint32_t gpio_errors;
	uint32_t pwm_errors;
	uint32_t tx_offline;		/* hb_send -ENOTCONN count */
};

#if IS_ENABLED(CONFIG_DEMO_HEARTBEAT_MICROROS)
void dds_get_diag(struct dds_diag *out);
#endif

void signal_chain_get_diag(struct signal_chain_diag *out);

#endif /* DDS_DIAG_H */
