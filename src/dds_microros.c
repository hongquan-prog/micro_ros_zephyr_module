/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Process Mission
 *
 * Real bidirectional heartbeat over micro-ROS and the ZVisor shared-memory
 * transport. The two directions use separate topics:
 *
 *   Zephyr -> Linux: /zephyr_int32_publisher
 *   Linux  -> Zephyr: /host_int32_publisher
 *
 * The micro-ROS entities are owned by one supervisor thread. hb_send() only
 * replaces the pending sequence number, so the signal-chain thread never
 * blocks in DDS and stale heartbeats do not accumulate while the Agent is
 * unavailable. When the Agent disconnects, the supervisor tears down and
 * recreates the session without restarting the GPIO/PWM threads.
 */

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include <microros_transports.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "heartbeat_dds.h"
#include "dds_diag.h"

LOG_MODULE_REGISTER(dds_microros, LOG_LEVEL_INF);

#define ZEPHYR_HEARTBEAT_TOPIC "zephyr_int32_publisher"
#define LINUX_HEARTBEAT_TOPIC   "host_int32_publisher"
#define NODE_NAME               "processone_zephyr_heartbeat"

#define AGENT_WAIT_INTERVAL_MS   250
#define AGENT_PING_INTERVAL_MS   500
#define AGENT_PING_TIMEOUT_MS    100
#define AGENT_PING_FAILURE_LIMIT 2
#define EXECUTOR_SPIN_TIMEOUT_NS RCL_MS_TO_NS(1)
#define MICROROS_STACK_SIZE      24576
#define REPLY_STACK_SIZE         1024
#define REPLY_THREAD_PRIORITY    4

struct microros_entities {
	rclc_support_t support;
	rcl_node_t node;
	rcl_publisher_t publisher;
	rcl_subscription_t subscriber;
	rclc_executor_t executor;
	bool support_ready;
	bool node_ready;
	bool publisher_ready;
	bool subscriber_ready;
	bool executor_ready;
};

static struct microros_entities entities;
static std_msgs__msg__Int32 tx_msg;
static std_msgs__msg__Int32 rx_msg;
static hb_recv_cb_t recv_cb;

K_SEM_DEFINE(recv_sem, 0, K_SEM_MAX_LIMIT);
static atomic_t received_linux_seq;
static atomic_t session_ready;
static atomic_t initialized;

/* Diagnostic counters (read via dds_get_diag()). */
static atomic_t session_rebuilds;
static atomic_t publish_failures;
static atomic_t ping_failures_total;

static struct k_spinlock tx_lock;
static uint32_t pending_tx_seq;
static bool tx_pending;

K_THREAD_STACK_DEFINE(microros_stack, MICROROS_STACK_SIZE);
static struct k_thread microros_thread_data;

K_THREAD_STACK_DEFINE(reply_stack, REPLY_STACK_SIZE);
static struct k_thread reply_thread_data;

static void log_entity_ready(uint8_t ordinal, const char *step)
{
	LOG_DBG("DDS entity %u/6 ready: %s", ordinal, step);
}

static void log_rcl_failure(uint8_t ordinal, const char *step,
			    rcl_ret_t result)
{
	rcl_error_string_t error = rcl_get_error_string();
	const char *detail = error.str[0] != '\0' ? error.str : "not set";

	LOG_WRN("DDS entity %u/6 failed: %s result=%d detail='%s'",
		ordinal, step, (int)result, detail);
}

static void entities_reset(struct microros_entities *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
	ctx->node = rcl_get_zero_initialized_node();
	ctx->publisher = rcl_get_zero_initialized_publisher();
	ctx->subscriber = rcl_get_zero_initialized_subscription();
	ctx->executor = rclc_executor_get_zero_initialized_executor();
}

static void cleanup_result(const char *operation, rcl_ret_t result)
{
	if (result == RCL_RET_OK) {
		return;
	}

	LOG_WRN("cleanup %s returned %d", operation, (int)result);
	rcl_reset_error();
}

static void destroy_entities(struct microros_entities *ctx)
{
	atomic_clear(&session_ready);

	if (ctx->support_ready) {
		rmw_context_t *rmw_context =
			rcl_context_get_rmw_context(&ctx->support.context);

		if (rmw_context != NULL) {
			(void)rmw_uros_set_context_entity_destroy_session_timeout(
				rmw_context, 0);
		}
	}

	if (ctx->executor_ready) {
		cleanup_result("executor", rclc_executor_fini(&ctx->executor));
	}
	if (ctx->subscriber_ready) {
		cleanup_result("subscriber",
			       rcl_subscription_fini(&ctx->subscriber, &ctx->node));
	}
	if (ctx->publisher_ready) {
		cleanup_result("publisher",
			       rcl_publisher_fini(&ctx->publisher, &ctx->node));
	}
	if (ctx->node_ready) {
		cleanup_result("node", rcl_node_fini(&ctx->node));
	}
	if (ctx->support_ready) {
		cleanup_result("support", rclc_support_fini(&ctx->support));
	}

	entities_reset(ctx);
}

static void reply_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		k_sem_take(&recv_sem, K_FOREVER);
		if (recv_cb != NULL) {
			recv_cb((uint32_t)atomic_get(&received_linux_seq));
		}
	}
}

static void subscription_callback(const void *message)
{
	const std_msgs__msg__Int32 *received = message;

	atomic_set(&received_linux_seq, (atomic_val_t)received->data);
	/* Preserve one GPIO/PWM action per real DDS reply.  Unlike a single
	 * k_work item, the counting semaphore does not coalesce replies that
	 * arrive while the handler is already running. */
	k_sem_give(&recv_sem);
}

static bool create_entities(struct microros_entities *ctx,
			    rcl_allocator_t *allocator)
{
	rcl_ret_t result;
	const char *failed_step = "support";
	uint8_t failed_ordinal = 1U;

	entities_reset(ctx);

	result = rclc_support_init(&ctx->support, 0, NULL, allocator);
	if (result != RCL_RET_OK) {
		goto fail;
	}
	ctx->support_ready = true;
	log_entity_ready(1U, "support");

	failed_step = "node";
	failed_ordinal = 2U;
	result = rclc_node_init_default(&ctx->node, NODE_NAME, "", &ctx->support);
	if (result != RCL_RET_OK) {
		goto fail;
	}
	ctx->node_ready = true;
	log_entity_ready(2U, "node");

	failed_step = "publisher";
	failed_ordinal = 3U;
	result = rclc_publisher_init_best_effort(
		&ctx->publisher, &ctx->node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		ZEPHYR_HEARTBEAT_TOPIC);
	if (result != RCL_RET_OK) {
		goto fail;
	}
	ctx->publisher_ready = true;
	log_entity_ready(3U, "publisher /" ZEPHYR_HEARTBEAT_TOPIC);

	failed_step = "subscriber";
	failed_ordinal = 4U;
	result = rclc_subscription_init_best_effort(
		&ctx->subscriber, &ctx->node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		LINUX_HEARTBEAT_TOPIC);
	if (result != RCL_RET_OK) {
		goto fail;
	}
	ctx->subscriber_ready = true;
	log_entity_ready(4U, "subscriber /" LINUX_HEARTBEAT_TOPIC);

	failed_step = "executor";
	failed_ordinal = 5U;
	result = rclc_executor_init(&ctx->executor, &ctx->support.context, 1,
				    allocator);
	if (result != RCL_RET_OK) {
		goto fail;
	}
	ctx->executor_ready = true;
	log_entity_ready(5U, "executor");

	failed_step = "executor subscription";
	failed_ordinal = 6U;
	result = rclc_executor_add_subscription(
		&ctx->executor, &ctx->subscriber, &rx_msg,
		subscription_callback, ON_NEW_DATA);
	if (result != RCL_RET_OK) {
		goto fail;
	}
	log_entity_ready(6U, "executor subscription");

	atomic_set(&session_ready, 1);
	LOG_INF("Agent connected: tx=/%s rx=/%s",
		ZEPHYR_HEARTBEAT_TOPIC, LINUX_HEARTBEAT_TOPIC);
	return true;

fail:
	log_rcl_failure(failed_ordinal, failed_step, result);
	rcl_reset_error();
	zephyr_transport_set_session_active(true);
	destroy_entities(ctx);
	zephyr_transport_set_session_active(false);
	return false;
}

static bool take_pending_sequence(uint32_t *sequence)
{
	k_spinlock_key_t key = k_spin_lock(&tx_lock);
	bool pending = tx_pending;

	if (pending) {
		*sequence = pending_tx_seq;
		tx_pending = false;
	}
	k_spin_unlock(&tx_lock, key);
	return pending;
}

static bool publish_pending_sequence(void)
{
	uint32_t sequence;
	rcl_ret_t result;

	if (!take_pending_sequence(&sequence)) {
		return true;
	}

	tx_msg.data = (int32_t)sequence;
	result = rcl_publish(&entities.publisher, &tx_msg, NULL);
	if (result == RCL_RET_OK) {
		return true;
	}

	atomic_inc(&publish_failures);
	LOG_WRN("publish failed (%d), reconnecting", (int)result);
	rcl_reset_error();
	return false;
}

static void microros_thread(void *arg1, void *arg2, void *arg3)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	LOG_INF("waiting for Micro-XRCE-DDS Agent");
	while (true) {
		int64_t next_ping_ms;
		uint8_t ping_failures = 0;
		bool session_error = false;

		while (!create_entities(&entities, &allocator)) {
			k_sleep(K_MSEC(AGENT_WAIT_INTERVAL_MS));
		}

		zephyr_transport_set_session_active(true);
		next_ping_ms = k_uptime_get() + AGENT_PING_INTERVAL_MS;

		while (!session_error) {
			rcl_ret_t result = rclc_executor_spin_some(
				&entities.executor, EXECUTOR_SPIN_TIMEOUT_NS);
			int64_t now_ms = k_uptime_get();

			if (result != RCL_RET_OK && result != RCL_RET_TIMEOUT) {
				LOG_WRN("executor failed (%d), reconnecting",
					(int)result);
				rcl_reset_error();
				session_error = true;
			}

			if (!session_error && !publish_pending_sequence()) {
				session_error = true;
			}

			if (!session_error && now_ms >= next_ping_ms) {
				if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) ==
				    RMW_RET_OK) {
					ping_failures = 0;
				} else {
					atomic_inc(&ping_failures_total);
					if (++ping_failures >=
					    AGENT_PING_FAILURE_LIMIT) {
						session_error = true;
					}
				}
				next_ping_ms = now_ms + AGENT_PING_INTERVAL_MS;
			}


			/* A 1 ms executor wait bounds reply handling latency while still
			 * blocking on the SHM RX doorbell instead of busy polling. */
			if (!session_error) {
				k_yield();
			}
		}

		LOG_WRN("Agent disconnected, rebuilding entities");
		atomic_inc(&session_rebuilds);
		destroy_entities(&entities);
		zephyr_transport_set_session_active(false);
		k_sleep(K_MSEC(AGENT_WAIT_INTERVAL_MS));
	}
}

int hb_init(hb_recv_cb_t cb)
{
	if (!atomic_cas(&initialized, 0, 1)) {
		return -EALREADY;
	}

	recv_cb = cb;
	entities_reset(&entities);
	atomic_clear(&received_linux_seq);
	atomic_clear(&session_ready);

	k_thread_create(&reply_thread_data, reply_stack,
			K_THREAD_STACK_SIZEOF(reply_stack), reply_thread,
			NULL, NULL, NULL, K_PRIO_PREEMPT(REPLY_THREAD_PRIORITY),
			0, K_NO_WAIT);

	rmw_uros_set_custom_transport(
		MICRO_ROS_FRAMING_REQUIRED,
		(void *)&default_params,
		zephyr_transport_open,
		zephyr_transport_close,
		zephyr_transport_write,
		zephyr_transport_read);

	k_thread_create(&microros_thread_data, microros_stack,
			K_THREAD_STACK_SIZEOF(microros_stack), microros_thread,
			NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);

	return 0;
}

int hb_send(uint32_t seq)
{
	k_spinlock_key_t key = k_spin_lock(&tx_lock);

	pending_tx_seq = seq;
	tx_pending = true;
	k_spin_unlock(&tx_lock, key);

	return atomic_get(&session_ready) != 0 ? 0 : -ENOTCONN;
}

void dds_get_diag(struct dds_diag *out)
{
	k_spinlock_key_t key;

	out->initialized = atomic_get(&initialized) != 0;
	out->session_ready = atomic_get(&session_ready) != 0;
	out->entity_ready_mask = 0;
	if (entities.support_ready) {
		out->entity_ready_mask |= BIT(0);
	}
	if (entities.node_ready) {
		out->entity_ready_mask |= BIT(1);
	}
	if (entities.publisher_ready) {
		out->entity_ready_mask |= BIT(2);
	}
	if (entities.subscriber_ready) {
		out->entity_ready_mask |= BIT(3);
	}
	if (entities.executor_ready) {
		out->entity_ready_mask |= BIT(4);
	}
	out->session_rebuilds = atomic_get(&session_rebuilds);
	out->publish_failures = atomic_get(&publish_failures);
	out->ping_failures_total = atomic_get(&ping_failures_total);

	key = k_spin_lock(&tx_lock);
	out->pending_tx_seq = pending_tx_seq;
	out->tx_pending = tx_pending;
	k_spin_unlock(&tx_lock, key);

	out->received_linux_seq = (int32_t)atomic_get(&received_linux_seq);
	out->recv_sem_count = k_sem_count_get(&recv_sem);

	k_thread_state_str(&microros_thread_data, out->supervisor_state,
			   sizeof(out->supervisor_state));
	k_thread_state_str(&reply_thread_data, out->reply_state,
			   sizeof(out->reply_state));
	k_thread_stack_space_get(&microros_thread_data,
				 &out->supervisor_stack_free);
	k_thread_stack_space_get(&reply_thread_data, &out->reply_stack_free);
}
