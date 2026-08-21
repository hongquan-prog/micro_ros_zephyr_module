#include <version.h>

#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(3,1,0)
#include <zephyr/kernel.h>
#else
#include <zephyr.h>
#endif

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>
#include <microros_transports.h>

#define AGENT_WAIT_INTERVAL_MS          250
#define AGENT_PING_INTERVAL_MS          500
#define AGENT_PING_TIMEOUT_MS           100
#define AGENT_PING_FAILURE_LIMIT        2
#define EXECUTOR_SPIN_TIMEOUT_NS        RCL_MS_TO_NS(10)
#define EXECUTOR_IDLE_INTERVAL_MS       50

struct microros_entities {
	rclc_support_t support;
	rcl_node_t node;
	rcl_publisher_t publisher;
	rcl_subscription_t subscriber;
	rcl_timer_t timer;
	rclc_executor_t executor;
	bool support_ready;
	bool node_ready;
	bool publisher_ready;
	bool subscriber_ready;
	bool timer_ready;
	bool executor_ready;
};

static struct microros_entities entities;
static std_msgs__msg__Int32 publish_msg;
static std_msgs__msg__Int32 receive_msg;
static bool session_error;

static void entities_reset(struct microros_entities *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
	ctx->node = rcl_get_zero_initialized_node();
	ctx->publisher = rcl_get_zero_initialized_publisher();
	ctx->subscriber = rcl_get_zero_initialized_subscription();
	ctx->timer = rcl_get_zero_initialized_timer();
	ctx->executor = rclc_executor_get_zero_initialized_executor();
}

static void cleanup_result(const char *operation, rcl_ret_t result)
{
	if (result == RCL_RET_OK) {
		return;
	}

	printf("micro-ROS: cleanup %s returned %d\n", operation, (int)result);
	rcl_reset_error();
}

static void destroy_entities(struct microros_entities *ctx)
{
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
	if (ctx->timer_ready) {
		cleanup_result("timer", rcl_timer_fini(&ctx->timer));
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
	session_error = false;
}

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	rcl_ret_t result;

	RCLC_UNUSED(last_call_time);
	if (timer == NULL || !entities.publisher_ready) {
		return;
	}

	result = rcl_publish(&entities.publisher, &publish_msg, NULL);
	if (result != RCL_RET_OK) {
		if (!session_error) {
			printf("micro-ROS: publish failed (%d), reconnecting\n",
			       (int)result);
		}
		session_error = true;
		rcl_reset_error();
		return;
	}

	publish_msg.data++;
}

static void subscription_callback(const void *message)
{
	const std_msgs__msg__Int32 *received = message;

	printf("micro-ROS: received from host: %d\n", received->data);
}

static bool create_entities(struct microros_entities *ctx,
			    rcl_allocator_t *allocator)
{
	rcl_ret_t result;
	const char *failed_step = "support";

	entities_reset(ctx);
	session_error = false;

	result = rclc_support_init(&ctx->support, 0, NULL, allocator);
	if (result != RCL_RET_OK) {
		goto fail;
	}
	ctx->support_ready = true;

	failed_step = "node";
	result = rclc_node_init_default(
		&ctx->node, "zephyr_int32_publisher", "", &ctx->support);
	if (result != RCL_RET_OK) {
		goto fail;
	}
	ctx->node_ready = true;

	failed_step = "publisher";
	result = rclc_publisher_init_default(
		&ctx->publisher,
		&ctx->node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"zephyr_int32_publisher");
	if (result != RCL_RET_OK) {
		goto fail;
	}
	ctx->publisher_ready = true;

	failed_step = "subscriber";
	result = rclc_subscription_init_default(
		&ctx->subscriber,
		&ctx->node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"host_int32_publisher");
	if (result != RCL_RET_OK) {
		goto fail;
	}
	ctx->subscriber_ready = true;

	failed_step = "timer";
	result = rclc_timer_init_default2(
		&ctx->timer,
		&ctx->support,
		RCL_MS_TO_NS(1000),
		timer_callback,
		true);
	if (result != RCL_RET_OK) {
		goto fail;
	}
	ctx->timer_ready = true;

	failed_step = "executor";
	result = rclc_executor_init(
		&ctx->executor, &ctx->support.context, 2, allocator);
	if (result != RCL_RET_OK) {
		goto fail;
	}
	ctx->executor_ready = true;

	failed_step = "executor timer";
	result = rclc_executor_add_timer(&ctx->executor, &ctx->timer);
	if (result != RCL_RET_OK) {
		goto fail;
	}

	failed_step = "executor subscription";
	result = rclc_executor_add_subscription(
		&ctx->executor,
		&ctx->subscriber,
		&receive_msg,
		subscription_callback,
		ON_NEW_DATA);
	if (result != RCL_RET_OK) {
		goto fail;
	}

	printf("micro-ROS: Agent connected, entities registered\n");
	return true;

fail:
	printf("micro-ROS: registration failed at %s (%d)\n",
	       failed_step, (int)result);
	rcl_reset_error();
	zephyr_transport_set_session_active(true);
	destroy_entities(ctx);
	zephyr_transport_set_session_active(false);
	return false;
}

int main(void)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();

	rmw_uros_set_custom_transport(
		MICRO_ROS_FRAMING_REQUIRED,
		(void *)&default_params,
		zephyr_transport_open,
		zephyr_transport_close,
		zephyr_transport_write,
		zephyr_transport_read);

	entities_reset(&entities);
	publish_msg.data = 0;
	receive_msg.data = 0;
	printf("micro-ROS: waiting for Agent\n");

	while (true) {
		int64_t next_ping_ms;
		uint8_t ping_failures = 0;

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
				printf("micro-ROS: executor failed (%d), reconnecting\n",
				       (int)result);
				rcl_reset_error();
				session_error = true;
			}

			if (!session_error && now_ms >= next_ping_ms) {
				if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) ==
				    RMW_RET_OK) {
					ping_failures = 0;
				} else if (++ping_failures >= AGENT_PING_FAILURE_LIMIT) {
					session_error = true;
				}
				next_ping_ms = now_ms + AGENT_PING_INTERVAL_MS;
			}

			if (!session_error) {
				k_sleep(K_MSEC(EXECUTOR_IDLE_INTERVAL_MS));
			}
		}

		printf("micro-ROS: Agent disconnected, rebuilding entities\n");
		destroy_entities(&entities);
		zephyr_transport_set_session_active(false);
		k_sleep(K_MSEC(AGENT_WAIT_INTERVAL_MS));
	}

	return 0;
}
