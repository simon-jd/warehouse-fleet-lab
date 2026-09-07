#!/usr/bin/env python3
"""Independent fail-closed velocity gate, bounded scan-delay injector and actuator-hold injector."""

import json
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from protocol import command_allowed
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Gate(Node):
    def __init__(self):
        super().__init__("command_gate")
        self.last_cmd = self.last_odom = self.last_permit = float("-inf")
        self.scan_stamp = float("-inf")
        self.permitted = False
        self.epoch = -1
        self.seq = -1
        self.command = Twist()
        self.queue = deque(maxlen=40)
        self.fault = None
        self.fault_end = 0.0
        self.was_allowed = None
        self.output = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_pub = self.create_publisher(
            LaserScan, "scan", qos_profile_sensor_data
        )
        self.events = self.create_publisher(String, "gate/events", 100)
        self.create_subscription(Twist, "cmd_vel_nav", self.cmd, 10)
        self.create_subscription(Odometry, "odom", self.odom, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, "scan_raw", self.scan, qos_profile_sensor_data
        )
        self.create_subscription(String, "permit", self.permit, 1)
        self.create_subscription(String, "inject_fault", self.inject, 10)
        self.timer = self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def emit(self, kind, **data):
        self.events.publish(
            String(
                data=json.dumps(
                    {"kind": kind, "sim_time": self.now(), **data}, allow_nan=False
                )
            )
        )

    def cmd(self, msg):
        self.command = msg
        self.last_cmd = time.monotonic()

    def odom(self, msg):
        self.last_odom = time.monotonic()

    def permit(self, msg):
        try:
            p = json.loads(msg.data)
            if (
                type(p["generation"]) is not int
                or type(p["sequence"]) is not int
                or type(p["allow"]) is not bool
            ):
                return
            if p["generation"] < self.epoch or p["sequence"] <= self.seq:
                return
            if (
                not math.isfinite(p["sim_time"])
                or abs(self.now() - p["sim_time"]) > 0.3
            ):
                return
            self.epoch = p["generation"]
            self.seq = p["sequence"]
            self.permitted = p["allow"]
            self.last_permit = time.monotonic()
        except (ValueError, KeyError, TypeError):
            return

    def scan(self, msg):
        delay = 1.0 if self.fault == "scan_delay" else 0.0
        if len(self.queue) == self.queue.maxlen:
            self.emit("scan_queue_overflow")
        self.queue.append((self.now() + delay, msg))

    def inject(self, msg):
        try:
            p = json.loads(msg.data)
            if p["kind"] not in ["scan_delay", "actuator_hold"] or self.fault:
                return
            duration = float(p["duration"])
            if not 0 < duration <= 10:
                return
            self.fault = p["kind"]
            self.fault_end = self.now() + duration
            self.emit("fault_started", fault=self.fault, duration=duration)
        except (ValueError, KeyError, TypeError):
            return

    def tick(self):
        now = self.now()
        wall = time.monotonic()
        if self.fault and now >= self.fault_end:
            self.emit("fault_cleared", fault=self.fault)
            self.fault = None
            self.queue.clear()  # Old delayed messages do not masquerade as newly recovered sensors.
        while self.queue and self.queue[0][0] <= now:
            _, scan = self.queue.popleft()
            self.scan_pub.publish(scan)
            self.scan_stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
        allowed = command_allowed(
            permitted=self.permitted,
            scan_age=now - self.scan_stamp,
            odom_age=wall - self.last_odom,
            command_age=wall - self.last_cmd,
            permit_age=wall - self.last_permit,
            hold=self.fault == "actuator_hold",
        )
        self.output.publish(self.command if allowed else Twist())
        if allowed != self.was_allowed:
            self.emit("gate_open" if allowed else "gate_closed", generation=self.epoch)
            self.was_allowed = allowed
        if int(wall * 10) % 5 == 0:
            self.events.publish(
                String(
                    data=json.dumps(
                        {
                            "kind": "gate_health",
                            "sim_time": now,
                            "scan_fresh": 0 <= now - self.scan_stamp < 0.65,
                            "odom_fresh": wall - self.last_odom < 0.5,
                            "fault": self.fault,
                            "allowed": allowed,
                        }
                    )
                )
            )


def main():
    rclpy.init()
    node = Gate()
    try:
        rclpy.spin(node)
    finally:
        node.output.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
