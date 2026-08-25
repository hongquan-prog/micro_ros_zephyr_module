/* SPDX-License-Identifier: Apache-2.0
 *
 * micro-ROS transport stress test (ZVisor shmem / OpenAMP).
 *
 * Guest -> host: publishes std_msgs/Int32 with a monotonically increasing
 * sequence number on "zephyr_stress" as fast as the transport allows
 * (single-slot protocol: write blocks until the peer frees the slot).
 *
 * Host -> guest: subscribes "host_stress" (host sends incrementing
 * sequences), counts received messages and sequence gaps.
 *
 * A separate thread prints stats every 2 s:
 *   STRESS tx=<n> tx_rate=<n>/s rx=<n> rx_lost=<n> pub=<f> spin=<f>
 *          wr=<n> wrwait=<n> isr=<n> badlen=<n> rd=<n> rdto=<n>
 * (wr/isr/badlen/rd are debug counters from the zvisor transport;
 * pub/spin are liveness flags of the main loop).
 */

#include <stdio.h>

#include <zephyr/kernel.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>
#include <microros_transports.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);for(;;){};}}

static rcl_publisher_t publisher;
static rcl_subscription_t subscriber;
static std_msgs__msg__Int32 tx_msg;
static std_msgs__msg__Int32 rx_msg;

static uint64_t tx_count;
static uint64_t rx_count;
static uint64_t rx_lost;
static int32_t rx_expected;
static bool rx_synced;

/* Liveness flags + transport debug counters (from the zvisor transport). */
static volatile bool in_spin;
static volatile bool in_publish;
extern uint32_t zvisor_dbg_write_calls;
extern uint32_t zvisor_dbg_write_waits;
extern uint32_t zvisor_dbg_isr;
extern uint32_t zvisor_dbg_isr_badlen;
extern uint32_t zvisor_dbg_read_calls;
extern uint32_t zvisor_dbg_read_timeouts;

static void subscription_callback(const void *msgin)
{
	const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgin;

	rx_count++;
	if (!rx_synced) {
		rx_synced = true;
		rx_expected = m->data + 1;
		return;
	}
	/* Gaps and duplicates/out-of-order both count as loss events. */
	if (m->data != rx_expected) {
		rx_lost += (m->data > rx_expected) ?
			   (uint64_t)(m->data - rx_expected) : 1;
		rx_expected = m->data + 1;
	} else {
		rx_expected++;
	}
}

static void stats_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint64_t last_tx = 0;
	int64_t last = k_uptime_get();

	while (1) {
		k_sleep(K_MSEC(2000));
		int64_t now = k_uptime_get();
		uint64_t rate = (tx_count - last_tx) * 1000 / (uint64_t)(now - last);
		printf("STRESS tx=%llu tx_rate=%llu/s rx=%llu rx_lost=%llu "
		       "pub=%d spin=%d wr=%u wrwait=%u isr=%u badlen=%u "
		       "rd=%u rdto=%u\n",
		       (unsigned long long)tx_count,
		       (unsigned long long)rate,
		       (unsigned long long)rx_count,
		       (unsigned long long)rx_lost,
		       in_publish, in_spin,
		       zvisor_dbg_write_calls, zvisor_dbg_write_waits,
		       zvisor_dbg_isr, zvisor_dbg_isr_badlen,
		       zvisor_dbg_read_calls, zvisor_dbg_read_timeouts);
		last = now;
		last_tx = tx_count;
	}
}

K_THREAD_DEFINE(stats_thread, 2048, stats_thread_fn, NULL, NULL, NULL,
		7, 0, 0);

int main(void)
{
	rmw_uros_set_custom_transport(
		MICRO_ROS_FRAMING_REQUIRED,
		(void *) &default_params,
		zephyr_transport_open,
		zephyr_transport_close,
		zephyr_transport_write,
		zephyr_transport_read
	);

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "zephyr_stress_node", "", &support));

	RCCHECK(rclc_publisher_init_best_effort(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"zephyr_stress"));

	RCCHECK(rclc_subscription_init_best_effort(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"host_stress"));

	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &rx_msg,
		&subscription_callback, ON_NEW_DATA));

	tx_msg.data = 0;

	printf("STRESS app started\n");

	while (1) {
		/* Non-blocking spin: pump any pending RX without throttling TX. */
		in_spin = true;
		rclc_executor_spin_some(&executor, 0);
		in_spin = false;

		in_publish = true;
		rcl_ret_t rc = rcl_publish(&publisher, &tx_msg, NULL);
		in_publish = false;
		if (rc == RCL_RET_OK) {
			tx_msg.data++;
			tx_count++;
		} else {
			/* Session not up yet (agent still starting). */
			k_msleep(10);
		}
	}

	return 0;
}
