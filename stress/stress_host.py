#!/usr/bin/env python3
"""Host side of the micro-ROS transport stress test.

Phase A (--duration seconds): subscribe "zephyr_stress" (guest publishes
incrementing Int32 sequences at max rate), count messages and gaps, print
the receive rate every 5 s.

Phase B: publish --burst incrementing messages to "host_stress" as fast as
possible, then drain until the guest stops acknowledging progress (it
prints its own rx counters on the serial console; the orchestrator script
greps them from the QEMU log).
"""

import argparse
import time

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32

rx_count = 0
rx_lost = 0
rx_expected = None
win_count = 0
win_start = None


def sub_cb(msg):
    global rx_count, rx_lost, rx_expected, win_count, win_start
    now = time.monotonic()
    if win_start is None:
        win_start = now
    rx_count += 1
    win_count += 1
    if rx_expected is None:
        rx_expected = msg.data + 1
    elif msg.data != rx_expected:
        if msg.data > rx_expected:
            rx_lost += msg.data - rx_expected
        else:
            rx_lost += 1  # duplicate / out of order
        rx_expected = msg.data + 1
    else:
        rx_expected += 1


def pump(node, seconds):
    """Busy-spin: process one callback per iteration. A single spin_once()
    handles at most one ready entity, so sleeping spins cannot keep up
    with a 5k msg/s stream and the DDS reader history overflows — that
    loss would be a test artifact, not a transport property."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0)


def main():
    global win_count
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=30.0,
                    help="seconds to measure guest->host throughput")
    ap.add_argument("--burst", type=int, default=500,
                    help="messages to send host->guest after phase A")
    ap.add_argument("--pace", type=float, default=500.0,
                    help="host->guest publish rate (msg/s); 0 = unthrottled")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("stress_host")

    # Best-effort reader matches both reliable and best-effort writers.
    sub_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1000,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Int32, "zephyr_stress", sub_cb, sub_qos)
    # Deep writer history: the whole burst must fit so keep-last overflow
    # cannot drop unacked samples at the DDS layer before the agent sees
    # them (we want to measure the XRCE/slot link, not FastDDS history).
    pub = node.create_publisher(Int32, "host_stress", QoSProfile(
        depth=max(100, args.burst * 2)))

    print(f"[host] phase A: measuring guest->host for {args.duration:.0f}s")
    end = time.monotonic() + args.duration
    last_report = time.monotonic()
    while time.monotonic() < end:
        pump(node, 0.05)
        now = time.monotonic()
        if now - last_report >= 5.0:
            rate = win_count / (now - last_report)
            print(f"[host] rx_total={rx_count} rate={rate:.0f}/s "
                  f"lost={rx_lost}")
            win_count = 0
            last_report = now

    phase_a_time = args.duration
    print(f"[host] phase A done: received={rx_count} lost={rx_lost} "
          f"avg_rate={rx_count / phase_a_time:.0f}/s")

    print(f"[host] phase B: sending burst of {args.burst} host->guest "
          f"(pace={args.pace:.0f}/s)")
    msg = Int32()
    t0 = time.monotonic()
    interval = 1.0 / args.pace if args.pace > 0 else 0.0
    for i in range(args.burst):
        msg.data = i
        pub.publish(msg)
        if interval > 0:
            # Busy-pump (instead of sleeping) so incoming guest->host
            # traffic keeps being consumed during the paced send.
            pump(node, max(0.0, t0 + (i + 1) * interval - time.monotonic()))
        else:
            rclpy.spin_once(node, timeout_sec=0)
    t1 = time.monotonic()
    print(f"[host] burst sent in {t1 - t0:.1f}s "
          f"({args.burst / max(t1 - t0, 1e-6):.0f}/s at DDS level)")

    # Drain: keep pumping so late guest->host traffic is still counted.
    pump(node, 10.0)

    print(f"[host] RESULT guest->host: received={rx_count} lost={rx_lost}")
    print(f"[host] RESULT host->guest: sent={args.burst} "
          f"(check guest STRESS rx= line in the QEMU log)")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
