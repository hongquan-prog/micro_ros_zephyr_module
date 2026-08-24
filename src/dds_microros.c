/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Process Mission
 *
 * Phase 2 real DDS heartbeat over micro-ROS.
 *
 * Single "heartbeat" topic carrying std_msgs/msg/Int32 in both
 * directions: zephyr publishes its sequence on every control point, and
 * subscribes to the same topic for the Linux replies.  DDS semantics
 * guarantee a publisher never receives its own samples, so the
 * subscriber only sees Linux-side data.
 *
 * The subscription callback defers the reply handling to the system
 * workqueue so the signal-chain control point keeps running in the same
 * context for both backends (stub and micro-ROS).
 */

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#include <rmw_microros/rmw_microros.h>
#include <microros_transports.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "heartbeat_dds.h"

LOG_MODULE_REGISTER(dds_microros, LOG_LEVEL_INF);

#define HEARTBEAT_TOPIC "heartbeat"
#define NODE_NAME       "zephyr_heartbeat"

#define RCCHECK(fn)							      \
	{								      \
		rcl_ret_t temp_rc = fn;					      \
		if ((temp_rc != RCL_RET_OK)) {				      \
			LOG_ERR("Failed status on line %d: %d. Aborting.",	      \
				__LINE__, (int)temp_rc);			      \
			for (;;) {					      \
			}						      \
		}							      \
	}

static rcl_publisher_t publisher;
static rcl_subscription_t subscriber;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

static std_msgs__msg__Int32 tx_msg;
static std_msgs__msg__Int32 rx_msg;

static hb_recv_cb_t recv_cb;
static struct k_work recv_work;
static struct k_mutex rx_lock;

K_THREAD_STACK_DEFINE(spin_stack, 4096);
static struct k_thread spin_tid;

/* Reply control point runs on the system workqueue, same as the stub. */
static void recv_work_handler(struct k_work *work)
{
	uint32_t linux_seq;

	ARG_UNUSED(work);

	k_mutex_lock(&rx_lock, K_FOREVER);
	linux_seq = (uint32_t)rx_msg.data;
	k_mutex_unlock(&rx_lock);

	if (recv_cb != NULL) {
		recv_cb(linux_seq);
	}
}

/* Subscription callback: executor (spin thread) context. */
static void subscription_callback(const void *msgin)
{
	const std_msgs__msg__Int32 *msg = msgin;

	k_mutex_lock(&rx_lock, K_FOREVER);
	rx_msg.data = msg->data;
	k_mutex_unlock(&rx_lock);

	k_work_submit(&recv_work);
}

static void spin_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		usleep(1000);
	}
}

int hb_init(hb_recv_cb_t cb)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();

	recv_cb = cb;

	k_work_init(&recv_work, recv_work_handler);
	k_mutex_init(&rx_lock);

	rmw_uros_set_custom_transport(
		MICRO_ROS_FRAMING_REQUIRED,
		(void *)&default_params,
		zephyr_transport_open,
		zephyr_transport_close,
		zephyr_transport_write,
		zephyr_transport_read);

	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		HEARTBEAT_TOPIC));

	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		HEARTBEAT_TOPIC));

	RCCHECK(rclc_executor_init(&executor, &support.context, 1,
				   &allocator));
	RCCHECK(rclc_executor_add_subscription(
		&executor, &subscriber, &rx_msg, subscription_callback,
		ON_NEW_DATA));

	tx_msg.data = 0;
	rx_msg.data = 0;

	k_thread_create(&spin_tid, spin_stack,
			K_THREAD_STACK_SIZEOF(spin_stack),
			spin_thread, NULL, NULL, NULL,
			CONFIG_NUM_PREEMPT_PRIORITIES - 2, 0,
			K_NO_WAIT);

	LOG_INF("micro-ROS heartbeat ready (topic '%s')", HEARTBEAT_TOPIC);

	return 0;
}

int hb_send(uint32_t seq)
{
	rcl_ret_t rc;

	tx_msg.data = (int32_t)seq;
	rc = rcl_publish(&publisher, &tx_msg, NULL);

	return (rc == RCL_RET_OK) ? 0 : -EIO;
}
