/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Process Mission
 *
 * "hb" shell: signal-chain / heartbeat business tuning.
 *
 * The control period drives the whole chain (GPIO1 tick, GPIO2/PWM1
 * control point, heartbeat send rate), so it is a business parameter
 * rather than a DDS diagnostic and lives here, not under "dds".
 *
 *   hb period [ms]   get/set the control period (1..1000 ms)
 *   hb freq [hz]     same thing expressed as a frequency (1..1000 Hz)
 *
 * Note: with the systick timer the effective minimum period is 1 ms;
 * sub-millisecond periods will be possible once the rk_timer counter
 * driver is brought back from the radxa-demo line.
 */

#include <errno.h>
#include <stdlib.h>

#include <zephyr/shell/shell.h>

#include "signal_chain.h"

static int cmd_hb_period(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1) {
		unsigned long ms = strtoul(argv[1], NULL, 0);
		int ret = signal_chain_set_period((uint32_t)ms);

		if (ret != 0) {
			shell_error(sh, "period must be 1..1000 ms");
			return ret;
		}
	}

	shell_print(sh, "control period: %u ms (tick %lu Hz, GPIO1 square %lu Hz)",
		    signal_chain_get_period(),
		    1000UL / signal_chain_get_period(),
		    1000UL / (2UL * signal_chain_get_period()));

	return 0;
}

static int cmd_hb_freq(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1) {
		unsigned long hz = strtoul(argv[1], NULL, 0);

		if (hz < 1UL || hz > 1000UL) {
			shell_error(sh, "frequency must be 1..1000 Hz");
			return -EINVAL;
		}

		(void)signal_chain_set_period((uint32_t)(1000UL / hz));
	}

	shell_print(sh, "control period: %u ms (tick %lu Hz, GPIO1 square %lu Hz)",
		    signal_chain_get_period(),
		    1000UL / signal_chain_get_period(),
		    1000UL / (2UL * signal_chain_get_period()));

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(hb_cmds,
	SHELL_CMD(period, NULL, "Get/set control period in ms (1..1000)", cmd_hb_period),
	SHELL_CMD(freq, NULL, "Get/set control frequency in Hz (1..1000)", cmd_hb_freq),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(hb, &hb_cmds, "Heartbeat/signal-chain tuning", NULL);
