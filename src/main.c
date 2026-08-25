/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Process Mission
 *
 * ProcessONE GPIO/PWM and DDS integration demo entry point.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int signal_chain_init(void);

int main(void)
{
	if (IS_ENABLED(CONFIG_DEMO_HEARTBEAT_MICROROS)) {
		LOG_INF("ProcessONE PWM/DDS demo starting (real SHM DDS)");
	} else {
		LOG_INF("ProcessONE PWM/DDS demo starting (stub heartbeat)");
	}

	if (signal_chain_init() != 0) {
		LOG_ERR("signal chain init failed");
		return -1;
	}

	return 0;
}
