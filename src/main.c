/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Process Mission
 *
 * PWM/DDS demo entry point (Phase 1: stubbed heartbeat).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int signal_chain_init(void);

int main(void)
{
	LOG_INF("PWM/DDS demo starting (Phase 1: stub heartbeat)");

	if (signal_chain_init() != 0) {
		LOG_ERR("signal chain init failed");
		return -1;
	}

	return 0;
}
