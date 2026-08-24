/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Process Mission
 *
 * Signal chain for the PWM/DDS demo:
 *
 *   1 ms systick (k_timer ISR)   -> GPIO1 toggle, wake worker thread
 *   worker thread (on sem)       -> GPIO2 toggle, PWM1 duty flip, hb_send
 *   reply callback (workqueue)   -> GPIO3 toggle, PWM2 duty flip
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
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "heartbeat_dds.h"

LOG_MODULE_REGISTER(signal_chain, LOG_LEVEL_INF);

#define PWM_PERIOD_US		100U	/* 10 kHz */
#define PWM_DUTY_LOW_US		25U	/* 25% */
#define PWM_DUTY_HIGH_US	75U	/* 75% */

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
static bool pwm1_high_duty;
static bool pwm2_high_duty;

/* --- control points ---------------------------------------------------- */

/* Control point 1: 1 ms systick ISR. */
static void tick_timer_expired(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	gpio_pin_toggle_dt(&gpio_probe1);
	k_sem_give(&tick_sem);
}

/* Control point 3: Linux heartbeat reply (workqueue context). */
static void heartbeat_reply(uint32_t linux_seq)
{
	ARG_UNUSED(linux_seq);

	gpio_pin_toggle_dt(&gpio_probe3);

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
		k_sem_take(&tick_sem, K_FOREVER);

		gpio_pin_toggle_dt(&gpio_probe2);

		pwm1_high_duty = !pwm1_high_duty;
		pwm_set_dt(&pwm1, PWM_USEC(PWM_PERIOD_US),
			   PWM_USEC(pwm1_high_duty ? PWM_DUTY_HIGH_US :
						     PWM_DUTY_LOW_US));

		zephyr_seq++;
		(void)hb_send(zephyr_seq);
	}
}

K_THREAD_DEFINE(signal_chain_tid, 1024,
		signal_chain_thread, NULL, NULL, NULL,
		CONFIG_NUM_PREEMPT_PRIORITIES - 1, 0, 0);

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

	/* 1 ms systick: CONFIG_SYS_CLOCK_TICKS_PER_SEC = 1000. */
	k_timer_init(&tick_timer, tick_timer_expired, NULL);
	k_sem_init(&tick_sem, 0, 1);

	ret = hb_init(heartbeat_reply);
	if (ret != 0) {
		LOG_ERR("hb_init failed (%d)", ret);
		return ret;
	}

	k_timer_start(&tick_timer, K_MSEC(1), K_MSEC(1));

	LOG_INF("signal chain started: PWM 10kHz 25%%<->75%%, GPIO probes low");

	return 0;
}
