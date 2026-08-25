/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Process Mission
 *
 * "dds" diagnostic shell for the PWM/DDS demo.
 *
 *   dds status               session/entity/slot/thread snapshot
 *   dds stats [reset]        signal-chain, DDS and transport counters
 *   dds shm [dump [n]]       shmem slot states; hexdump of the rsp window
 *
 * The slot states are read straight from the zvisor-shmem windows (the
 * windows are mapped cache-none device memory and the GPA is fixed by
 * the devicetree), so the transport module does not need to expose
 * anything for this shell to work.  The zvisor_dbg_* counters are plain
 * global symbols in the transport, referenced the same way the stress
 * app does.
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "dds_diag.h"

#define ZV_NODE		DT_NODELABEL(zv_shm)
#define REQ_GPA		((uint32_t)DT_REG_ADDR(ZV_NODE))
#define RSP_GPA		(REQ_GPA + (uint32_t)DT_REG_SIZE(ZV_NODE) / 2U)

/* Frame layout must match the zvisor_shmem transport: [len][state][payload]. */
#define FRAME_OFF_LEN		0U
#define FRAME_OFF_STATE		4U
#define FRAME_OFF_PAYLOAD	8U

/* Cap the hexdump to keep the console output manageable. */
#define DUMP_DEFAULT_BYTES	64U
#define DUMP_MAX_BYTES		512U

#if IS_ENABLED(CONFIG_DEMO_HEARTBEAT_MICROROS)
/* Transport debug counters (global symbols, see microros_transports.c). */
extern uint32_t zvisor_dbg_write_calls;
extern uint32_t zvisor_dbg_write_waits;
extern uint32_t zvisor_dbg_isr;
extern uint32_t zvisor_dbg_isr_badlen;
extern uint32_t zvisor_dbg_read_calls;
extern uint32_t zvisor_dbg_read_timeouts;
#endif

static const char *slot_name(uint32_t state)
{
	return state != 0U ? "BUSY" : "FREE";
}

static int cmd_dds_status(const struct shell *sh, size_t argc, char **argv)
{
	struct signal_chain_diag sc;
	uint32_t req_state, req_len, rsp_state, rsp_len;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	req_state = sys_read32(REQ_GPA + FRAME_OFF_STATE);
	req_len = sys_read32(REQ_GPA + FRAME_OFF_LEN);
	rsp_state = sys_read32(RSP_GPA + FRAME_OFF_STATE);
	rsp_len = sys_read32(RSP_GPA + FRAME_OFF_LEN);

	shell_print(sh, "shm slots: req@0x%x %s(%uB)  rsp@0x%x %s(%uB)",
		    REQ_GPA, slot_name(req_state), req_len,
		    RSP_GPA, slot_name(rsp_state), rsp_len);

	signal_chain_get_diag(&sc);
	shell_print(sh, "signal chain: ticks=%u control=%u missed=%u tx=%u rx=%u last_linux=%u",
		    sc.timer_ticks, sc.control_count, sc.missed_ticks,
		    sc.tx_seq, sc.rx_count, sc.last_linux_seq);

#if IS_ENABLED(CONFIG_DEMO_HEARTBEAT_MICROROS)
	{
		struct dds_diag d;

		dds_get_diag(&d);
		shell_print(sh, "dds: initialized=%d session=%s rebuilds=%u publish_fail=%u ping_fail=%u",
			    d.initialized, d.session_ready ? "UP" : "DOWN",
			    d.session_rebuilds, d.publish_failures,
			    d.ping_failures_total);
		shell_print(sh, "entities support..subscription: %d%d%d%d%d%d",
			    !!(d.entity_ready_mask & BIT(0)),
			    !!(d.entity_ready_mask & BIT(1)),
			    !!(d.entity_ready_mask & BIT(2)),
			    !!(d.entity_ready_mask & BIT(3)),
			    !!(d.entity_ready_mask & BIT(4)),
			    !!(d.entity_ready_mask & BIT(5)));
		shell_print(sh, "tx pending=%s(seq=%u)  rx last=%d queued=%u",
			    d.tx_pending ? "yes" : "no", d.pending_tx_seq,
			    d.received_linux_seq, d.recv_sem_count);
		shell_print(sh, "threads: supervisor[%s free=%uB] reply[%s free=%uB]",
			    d.supervisor_state, d.supervisor_stack_free,
			    d.reply_state, d.reply_stack_free);
	}
#else
	shell_print(sh, "dds: backend=stub (no DDS session)");
#endif

	return 0;
}

static int cmd_dds_stats(const struct shell *sh, size_t argc, char **argv)
{
	struct signal_chain_diag sc;

	if (argc > 1 && strcmp(argv[1], "reset") == 0) {
#if IS_ENABLED(CONFIG_DEMO_HEARTBEAT_MICROROS)
		zvisor_dbg_write_calls = 0;
		zvisor_dbg_write_waits = 0;
		zvisor_dbg_isr = 0;
		zvisor_dbg_isr_badlen = 0;
		zvisor_dbg_read_calls = 0;
		zvisor_dbg_read_timeouts = 0;
#endif
		shell_print(sh, "transport counters reset");
		return 0;
	}

	signal_chain_get_diag(&sc);
	shell_print(sh, "signal chain: tx=%u rx=%u last_linux=%u missed=%u gpio_err=%u pwm_err=%u tx_offline=%u",
		    sc.tx_seq, sc.rx_count, sc.last_linux_seq, sc.missed_ticks,
		    sc.gpio_errors, sc.pwm_errors, sc.tx_offline);

#if IS_ENABLED(CONFIG_DEMO_HEARTBEAT_MICROROS)
	{
		struct dds_diag d;

		dds_get_diag(&d);
		shell_print(sh, "dds: rebuilds=%u publish_fail=%u ping_fail=%u",
			    d.session_rebuilds, d.publish_failures,
			    d.ping_failures_total);
		shell_print(sh, "transport: wr=%u wr_wait=%u isr=%u isr_badlen=%u rd=%u rd_to=%u",
			    zvisor_dbg_write_calls, zvisor_dbg_write_waits,
			    zvisor_dbg_isr, zvisor_dbg_isr_badlen,
			    zvisor_dbg_read_calls, zvisor_dbg_read_timeouts);
	}
#else
	shell_print(sh, "transport: n/a (stub backend)");
#endif

	return 0;
}

static void shm_dump(const struct shell *sh, uint32_t bytes)
{
	uint32_t i;

	for (i = 0; i < bytes; i++) {
		uint8_t byte = sys_read8(RSP_GPA + FRAME_OFF_PAYLOAD + i);

		if ((i % 16U) == 0U) {
			if (i != 0U) {
				shell_print(sh, "");
			}
			shell_fprintf(sh, SHELL_NORMAL, "%04x: ", i);
		}
		shell_fprintf(sh, SHELL_NORMAL, "%02x ", byte);
	}
	shell_print(sh, "");
}

static int cmd_dds_shm(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t req_state, req_len, rsp_state, rsp_len;

	req_state = sys_read32(REQ_GPA + FRAME_OFF_STATE);
	req_len = sys_read32(REQ_GPA + FRAME_OFF_LEN);
	rsp_state = sys_read32(RSP_GPA + FRAME_OFF_STATE);
	rsp_len = sys_read32(RSP_GPA + FRAME_OFF_LEN);

	shell_print(sh, "req@0x%x: %s len=%u", REQ_GPA, slot_name(req_state),
		    req_len);
	shell_print(sh, "rsp@0x%x: %s len=%u", RSP_GPA, slot_name(rsp_state),
		    rsp_len);

	if (argc > 1 && strcmp(argv[1], "dump") == 0) {
		unsigned long bytes = DUMP_DEFAULT_BYTES;

		if (argc > 2) {
			bytes = strtoul(argv[2], NULL, 0);
		}
		if (bytes == 0UL || bytes > DUMP_MAX_BYTES) {
			shell_error(sh, "dump length must be 1..%u",
				    DUMP_MAX_BYTES);
			return -EINVAL;
		}

		shell_print(sh, "rsp payload dump (%lu bytes):", bytes);
		shm_dump(sh, (uint32_t)bytes);
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(dds_cmds,
	SHELL_CMD(status, NULL,
		  "Session/entity/slot/thread status snapshot", cmd_dds_status),
	SHELL_CMD(stats, NULL,
		  "Signal-chain, DDS and transport counters (stats reset clears transport counters)",
		  cmd_dds_stats),
	SHELL_CMD(shm, NULL,
		  "Shmem slot states (shm dump [n] hexdumps the rsp window)",
		  cmd_dds_shm),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(dds, &dds_cmds, "DDS diagnostics", NULL);
