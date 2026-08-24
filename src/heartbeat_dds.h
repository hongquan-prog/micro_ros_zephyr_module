/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Process Mission
 *
 * DDS heartbeat abstraction layer.  Phase 1 is backed by dds_stub.c;
 * Phase 2 will plug in the real micro-ROS implementation.
 */

#ifndef HEARTBEAT_DDS_H
#define HEARTBEAT_DDS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Callback invoked when a Linux heartbeat reply arrives. */
typedef void (*hb_recv_cb_t)(uint32_t linux_seq);

/** Initialize the heartbeat channel and register the reply callback. */
int hb_init(hb_recv_cb_t cb);

/** Send a zephyr heartbeat with the given sequence number. */
int hb_send(uint32_t seq);

#ifdef __cplusplus
}
#endif

#endif /* HEARTBEAT_DDS_H */
