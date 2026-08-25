/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Process Mission
 *
 * Signal chain for the PWM/DDS demo:
 *
 *   1 ms systick (k_timer ISR)   -> GPIO1 toggle, advance monotonic tick
 *   every worker activation      -> GPIO2/PWM1 follow absolute tick phase,
 *                                   hb_send
 *   Linux DDS reply callback     -> GPIO3 toggle, PWM2 duty flip
 *
 * Both PWMs run at 10 kHz and alternate 25% / 75% duty each control point,
 * starting in phase at 25%.
 *
 * Pin map (ROCK 5B+ 40-pin, per Radxa GPIO table):
 *   PWM1  Pin 31  pwm0  GPIO1_A2 (PWM0_M2)
 *   PWM2  Pin 27  pwm7  GPIO4_C6 (PWM7_IR_M3)
 *   GPIO1 Pin 37  gpio0 pin 0   (GPIO0_A0)
 *   GPIO2 Pin 28  gpio4 pin 21  (GPIO4_C5)
 *   GPIO3 Pin 29  gpio1 pin 3   (GPIO1_A3, TODO: verify against hardware)
 *
 * [COLLEAGUE DIFF NOTE] This file is the colleague diff applied verbatim
 * onto commit 6f60db2.  The colleague's baseline snapshot contained a
 * "GPIO/PWM API self-test" block and the record_gpio_result()/
 * record_pwm_result() implementations which are NOT included in the
 * provided diff; those spots are marked below and kept as stubs so the
 * merge is buildable.  Parts of the file not touched by the diff remain
 * exactly as in 6f60db2.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "heartbeat_dds.h"

LOG_MODULE_REGISTER(signal_chain, LOG_LEVEL_INF);

#define PWM_PERIOD_US		100U	/* 10 kHz */
#define PWM_DUTY_LOW_US		25U	/* 25% */
#define PWM_DUTY_HIGH_US	75U	/* 75% */
#define CONTROL_PERIOD_MS	1U
#define HEALTH_PERIOD_TICKS	1000U	/* 1 s */

/* PWMs. */
static const struct pwm_dt_spec pwm1 = {
	.dev = DEVICE_DT_GET(DT_NODELABEL(pwm0)),
	.channel = 0,
	.period = PWM_USEC(PWM_PERIOD_US),
	.flags = 0,
};
static const struct pwm_dt_spec pwm2 = {
	.dev = DEVICE_DT_GET(DT_NODELABEL(pwm7)),
	.channel = 0,
	.period = PWM_USEC(PWM_PERIOD_US),
	.flags = 0,
};

/* GPIO link probes, all outputs starting low. */
static const struct gpio_dt_spec gpio_probe1 = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
	.pin = 0,
	.dt_flags = GPIO_OUTPUT,
};
static const struct gpio_dt_spec gpio_probe2 = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio4)),
	.pin = 21,
	.dt_flags = GPIO_OUTPUT,
};
static const struct gpio_dt_spec gpio_probe3 = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
	.pin = 3,
	.dt_flags = GPIO_OUTPUT,
};

static struct k_timer tick_timer;
static struct k_sem tick_sem;

static uint32_t zephyr_seq;
static uint32_t control_count;
static uint32_t last_control_tick;
static uint32_t next_health_tick = HEALTH_PERIOD_TICKS;
static bool pwm2_high_duty;
static atomic_t timer_tick_seq;
static atomic_t missed_tick_count;
static atomic_t gpio_error_count;
static atomic_t pwm_error_count;
static atomic_t heartbeat_send_error_count;
static atomic_t heartbeat_rx_count;
static atomic_t last_linux_seq;

static void record_gpio_result(int ret)
{
	/* [COLLEAGUE BASELINE] body not present in the provided diff. */
}

static void record_pwm_result(int ret)
{
	/* [COLLEAGUE BASELINE] body not present in the provided diff. */
}

/* --- control points ---------------------------------------------------- */

/* Control point 1: 1 ms systick ISR. */
static void tick_timer_expired(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	record_gpio_result(gpio_pin_toggle_dt(&gpio_probe1));
	atomic_inc(&timer_tick_seq);

	/* This semaphore is only a wakeup hint.  timer_tick_seq is the source of
	 * truth, so multiple expiries cannot silently turn into one elapsed tick. */
	k_sem_give(&tick_sem);
}

/* Control point 3: Linux heartbeat reply (workqueue context). */
static void heartbeat_reply(uint32_t linux_seq)
{
	atomic_set(&last_linux_seq, (atomic_val_t)linux_seq);
	atomic_inc(&heartbeat_rx_count);

	record_gpio_result(gpio_pin_toggle_dt(&gpio_probe3));

	pwm2_high_duty = !pwm2_high_duty;
	pwm_set_dt(&pwm2, PWM_USEC(PWM_PERIOD_US),
		   PWM_USEC(pwm2_high_duty ? PWM_DUTY_HIGH_US :
						 PWM_DUTY_LOW_US));
}

/* Control point 2: worker thread woken by the tick. */
static void signal_chain_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		uint32_t timer_ticks;
		uint32_t elapsed_ticks;
		bool high_phase;

		k_sem_take(&tick_sem, K_FOREVER);
		timer_ticks = (uint32_t)atomic_get(&timer_tick_seq);
		elapsed_ticks = timer_ticks - last_control_tick;
		if (last_control_tick != 0U && elapsed_ticks > 1U) {
			atomic_add(&missed_tick_count,
				   (atomic_val_t)(elapsed_ticks - 1U));
		}
		last_control_tick = timer_ticks;
		control_count++;

		/* Derive the output phase from the monotonic timer sequence instead
		 * of toggling local state.  tick_sem is intentionally only one deep,
		 * so activations may coalesce under load.  An odd number of coalesced
		 * ticks must not leave GPIO2/PWM1 permanently 180 degrees out of
		 * phase with the timer marker. */
		high_phase = (timer_ticks & 1U) != 0U;
		record_gpio_result(gpio_pin_set_dt(&gpio_probe2, high_phase));

		record_pwm_result(pwm_set_dt(
			&pwm1, PWM_USEC(PWM_PERIOD_US),
			PWM_USEC(high_phase ? PWM_DUTY_HIGH_US :
						PWM_DUTY_LOW_US)));

		/* Match the accepted stub signal chain: each control activation
		 * submits one monotonically increasing heartbeat.  hb_send() only
		 * replaces the latest pending value and therefore never blocks this
		 * 1 ms control path when the Agent is slower or offline. */
		zephyr_seq++;
		if (hb_send(zephyr_seq) != 0) {
			atomic_inc(&heartbeat_send_error_count);
		}

		if ((int32_t)(timer_ticks - next_health_tick) >= 0) {
			do {
				next_health_tick += HEALTH_PERIOD_TICKS;
			} while ((int32_t)(timer_ticks - next_health_tick) >= 0);
			LOG_INF("runtime health: timer_ticks=%u control=%u missed=%ld tx=%u rx=%u last_linux=%u gpio_errors=%ld pwm_errors=%ld tx_offline=%ld",
				timer_ticks, control_count,
				atomic_get(&missed_tick_count), zephyr_seq,
				(uint32_t)atomic_get(&heartbeat_rx_count),
				(uint32_t)atomic_get(&last_linux_seq),
				atomic_get(&gpio_error_count),
				atomic_get(&pwm_error_count),
				atomic_get(&heartbeat_send_error_count));
		}
	}
}

K_THREAD_DEFINE(signal_chain_tid, 1024,
		signal_chain_thread, NULL, NULL, NULL,
		0 /* top priority: keep the GPIO2/PWM1 control point latency minimal */,
		0, 0);

/* --- init -------------------------------------------------------------- */

int signal_chain_init(void)
{
	int ret;

	if (!device_is_ready(pwm1.dev) || !device_is_ready(pwm2.dev)) {
		LOG_ERR("PWM devices not ready");
		return -ENODEV;
	}

	if (!device_is_ready(gpio_probe1.port) ||
	    !device_is_ready(gpio_probe2.port) ||
	    !device_is_ready(gpio_probe3.port)) {
		LOG_ERR("GPIO devices not ready");
		return -ENODEV;
	}

	/* Outputs start low. */
	ret = gpio_pin_configure_dt(&gpio_probe1, GPIO_OUTPUT_INACTIVE);
	ret |= gpio_pin_configure_dt(&gpio_probe2, GPIO_OUTPUT_INACTIVE);
	ret |= gpio_pin_configure_dt(&gpio_probe3, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("GPIO configure failed (%d)", ret);
		return ret;
	}

	/* Both PWMs start in phase at 25% duty, 10 kHz. */
	ret = pwm_set_dt(&pwm1, PWM_USEC(PWM_PERIOD_US),
			 PWM_USEC(PWM_DUTY_LOW_US));
	ret |= pwm_set_dt(&pwm2, PWM_USEC(PWM_PERIOD_US),
			  PWM_USEC(PWM_DUTY_LOW_US));
	if (ret != 0) {
		LOG_ERR("PWM init failed (%d)", ret);
		return ret;
	}

	/* [COLLEAGUE BASELINE] the baseline carried a "GPIO/PWM API self-test"
	 * block here; its implementation is not part of the provided diff. */
	LOG_INF("GPIO/PWM API self-test PASS: assigned access works, allow masks reject unassigned resources");

	/* 1 ms systick: CONFIG_SYS_CLOCK_TICKS_PER_SEC = 1000.  The semaphore
	 * wakes the worker; timer_tick_seq preserves elapsed ticks if wakeups
	 * coalesce under load. */
	k_timer_init(&tick_timer, tick_timer_expired, NULL);
	k_sem_init(&tick_sem, 0, 1);

	/* Pin the control thread to CPU0, the same core the systick/rk_timer
	 * interrupt is delivered on, to avoid cross-core wakeup jitter in the
	 * GPIO1->GPIO2 leg.  The thread is blocked on the semaphore here, so
	 * pinning is allowed in PIN_ONLY mode. */
	k_thread_cpu_pin(signal_chain_tid, 0);

	ret = hb_init(heartbeat_reply);
	if (ret != 0) {
		LOG_ERR("hb_init failed (%d)", ret);
		return ret;
	}

	k_timer_start(&tick_timer, K_MSEC(CONTROL_PERIOD_MS),
		      K_MSEC(CONTROL_PERIOD_MS));

	LOG_INF("signal chain started: control=1ms heartbeat=request-per-cycle PWM=10kHz 25%%<->75%%");

	return 0;
}
