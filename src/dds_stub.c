/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Process Mission
 *
 * Phase 1 DDS stub: simulates the Linux round trip with a one-shot timer.
 * Delay = 0.2 ms fixed + 0..0.2 ms pseudo-random jitter (an LCG; no true
 * randomness needed, the jitter only has to be observable on the logic
 * analyzer).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "heartbeat_dds.h"

LOG_MODULE_REGISTER(dds_stub, LOG_LEVEL_INF);

static hb_recv_cb_t recv_cb;
static uint32_t linux_seq;
static uint32_t lcg_state;

static struct k_timer reply_timer;
static struct k_work reply_work;

static void reply_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	linux_seq++;
	if (recv_cb != NULL) {
		recv_cb(linux_seq);
	}
}

static void reply_timer_expired(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	k_work_submit(&reply_work);
}

int hb_init(hb_recv_cb_t cb)
{
	recv_cb = cb;
	linux_seq = 0;
	lcg_state = k_cycle_get_32() | 1U;

	k_timer_init(&reply_timer, reply_timer_expired, NULL);
	k_work_init(&reply_work, reply_work_handler);

	return 0;
}

int hb_send(uint32_t seq)
{
	uint32_t jitter_us;
	uint32_t delay_us;

	ARG_UNUSED(seq);

	/* LCG pseudo-random jitter in [0, 200) us. */
	lcg_state = lcg_state * 1103515245U + 12345U;
	jitter_us = (lcg_state >> 16) % 200U;

	/* 0.2 ms fixed round-trip + up to 0.2 ms jitter. */
	delay_us = 200U + jitter_us;

	k_timer_start(&reply_timer, K_USEC(delay_us), K_NO_WAIT);

	return 0;
}
