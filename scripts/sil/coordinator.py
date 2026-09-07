#!/usr/bin/env python3
"""Bounded Nav2 mission coordinator. A cancellation never releases the transit lease."""

import json
import math
import os
import signal
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from protocol import TransitLease
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from warehouse_fleet_ros.srv import AllocateTasks
from world import DELIVERIES, STARTS, cell


class Coordinator(Node):
    def __init__(self, robots, fault, event_sink):
        super().__init__(
            "fleet_coordinator",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)],
        )
        self.robots = robots
        self.fault = fault
        self.event_sink = event_sink
        self.lease = TransitLease()
        self.enabled = False
        self.sequence = 0
        self.injected = False
        self.completed = []
        self.pose = {}
        self.health = {}
        self.actions = {}
        self.permits = {}
        self.injectors = {}
        self.lifecycle = {}
        self.timer = self.create_timer(
            0.08, self.heartbeat, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.allocator = self.create_client(AllocateTasks, "/fleet/allocate")
        for robot in range(1, robots + 1):
            ns = f"/robot{robot}"
            self.actions[robot] = ActionClient(
                self, NavigateToPose, ns + "/navigate_to_pose"
            )
            self.permits[robot] = self.create_publisher(String, ns + "/permit", 1)
            self.injectors[robot] = self.create_publisher(
                String, ns + "/inject_fault", 10
            )
            self.create_subscription(
                PoseWithCovarianceStamped,
                ns + "/amcl_pose",
                lambda msg, r=robot: self.on_pose(r, msg),
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
            self.create_subscription(
                String,
                ns + "/gate/events",
                lambda msg, r=robot: self.on_health(r, msg),
                100,
            )
            for name in (
                "amcl",
                "bt_navigator",
                "controller_server",
                "collision_monitor",
            ):
                self.lifecycle[robot, name] = self.create_client(
                    GetState, ns + "/" + name + "/get_state"
                )

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def event(self, kind, **data):
        self.event_sink(
            {
                "kind": kind,
                "sim_time": self.now(),
                "wall_time": time.monotonic(),
                **data,
            }
        )

    def on_pose(self, robot, msg):
        self.pose[robot] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            time.monotonic(),
            msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        )

    def on_health(self, robot, msg):
        data = json.loads(msg.data)
        if data["kind"] == "gate_health":
            self.health[robot] = (data, time.monotonic())

    def heartbeat(self):
        self.sequence += 1
        for robot, publisher in self.permits.items():
            publisher.publish(
                String(
                    data=json.dumps(
                        {
                            "generation": self.lease.generation,
                            "sequence": self.sequence,
                            "allow": self.enabled and robot == self.lease.owner,
                            "sim_time": self.now(),
                        }
                    )
                )
            )

    def wait(self, predicate, timeout, description):
        end = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= end:
                raise TimeoutError(description)
            rclpy.spin_once(self, timeout_sec=0.03)

    def future(self, future, timeout, description):
        self.wait(future.done, timeout, description)
        return future.result()

    def sensors_ready(self, robot):
        data, received = self.health.get(robot, ({}, 0))
        return (
            time.monotonic() - received < 1
            and data.get("scan_fresh")
            and data.get("odom_fresh")
            and not data.get("fault")
        )

    def ready(self, robot, timeout=90):
        end = time.monotonic() + timeout
        self.wait(
            lambda: (
                self.actions[robot].server_is_ready()
                and robot in self.pose
                and self.sensors_ready(robot)
            ),
            timeout,
            f"robot{robot} action/localization/sensors ready",
        )
        stable_rounds = 0
        while time.monotonic() < end:
            clients = [
                client for (r, _), client in self.lifecycle.items() if r == robot
            ]
            active = all(client.service_is_ready() for client in clients)
            if active:
                futures = [client.call_async(GetState.Request()) for client in clients]
                states = [
                    self.future(future, 5, "lifecycle response").current_state.id
                    for future in futures
                ]
                active = all(state == 3 for state in states)
            stable_rounds = (
                stable_rounds + 1 if active and self.sensors_ready(robot) else 0
            )
            if stable_rounds >= 2:
                break
            next_round = time.monotonic() + 0.3
            self.wait(
                lambda next_round=next_round: time.monotonic() >= next_round,
                2,
                "lifecycle stabilization",
            )
        else:
            raise TimeoutError(f"robot{robot} lifecycle stabilization")
        self.event("robot_ready", robot=robot)

    def inject(self, robot):
        self.injected = True
        if self.fault == "nav_restart":
            # Only inside our private PID namespace. Never kill arbitrary host processes.
            if not Path("/.dockerenv").exists():
                raise RuntimeError(
                    "restart injector requires an isolated Docker container"
                )
            matches = []
            for proc in Path("/proc").iterdir():
                if not proc.name.isdigit():
                    continue
                try:
                    args = (proc / "cmdline").read_bytes().split(b"\0")
                    if (
                        args
                        and args[0].endswith(b"/controller_server")
                        and f"__ns:=/robot{robot}".encode() in args
                    ):
                        matches.append(int(proc.name))
                except (FileNotFoundError, PermissionError, ProcessLookupError):
                    pass
            if len(matches) != 1:
                raise RuntimeError(
                    f"expected one owned controller process, got {matches}"
                )
            self.event(
                "fault_requested",
                fault=self.fault,
                robot=robot,
                pid=matches[0],
                generation=self.lease.generation,
            )
            os.kill(matches[0], signal.SIGKILL)
        else:
            self.event(
                "fault_requested",
                fault=self.fault,
                robot=robot,
                duration=4.0,
                generation=self.lease.generation,
            )
            self.injectors[robot].publish(
                String(data=json.dumps({"kind": self.fault, "duration": 4.0}))
            )

    def navigate(self, robot, target, phase):
        for attempt in range(1, 4):
            self.ready(robot)
            self.enabled = True
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = target
            yaw = math.pi if robot == 2 and phase != "pickup" else 0.0
            goal.pose.pose.orientation.z = math.sin(yaw / 2)
            goal.pose.pose.orientation.w = math.cos(yaw / 2)
            epoch = self.lease.generation
            self.event(
                "goal_sent",
                robot=robot,
                generation=epoch,
                target=target,
                phase=phase,
                attempt=attempt,
            )
            handle = self.future(
                self.actions[robot].send_goal_async(goal), 15, "goal acknowledgment"
            )
            if not handle.accepted:
                self.event("goal_rejected", robot=robot, phase=phase, attempt=attempt)
                self.enabled = False
                continue
            result_future = handle.get_result_async()
            started = self.now()
            end = time.monotonic() + 180
            cancelled = False
            while not result_future.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.03)
                if (
                    self.fault != "none"
                    and not self.injected
                    and robot == 1
                    and phase == "entry"
                    and self.now() - started > 2
                ):
                    self.inject(robot)
                    if self.fault == "scan_delay":
                        # Exercise acknowledged cancellation + retry under the SAME lease.
                        self.enabled = False
                        response = self.future(
                            handle.cancel_goal_async(), 10, "cancel acknowledgment"
                        )
                        if not response.goals_canceling:
                            raise RuntimeError(
                                "cancellation not acknowledged; retaining lease"
                            )
                        self.event("cancel_acknowledged", robot=robot, generation=epoch)
                        cancelled = True
            if not result_future.done():
                self.enabled = False
                self.future(
                    handle.cancel_goal_async(),
                    10,
                    "deadline cancellation acknowledgment",
                )
                raise TimeoutError("goal deadline exceeded; retaining lease")
            result = result_future.result()
            self.event(
                "goal_result",
                robot=robot,
                generation=epoch,
                phase=phase,
                attempt=attempt,
                status=result.status,
            )
            if self.lease.owner != robot or self.lease.generation != epoch:
                raise RuntimeError("stale action result")
            if result.status == 4 and not cancelled:
                return
            self.enabled = False
            # Sensors and lifecycle must recover before retry; permit remains denied.
            pause_until = self.now() + (5.0 if cancelled else 3.0)
            self.wait(
                lambda pause_until=pause_until: self.now() >= pause_until,
                30,
                "recovery pause",
            )
            self.event(
                "retry", robot=robot, generation=epoch, phase=phase, attempt=attempt
            )
        raise RuntimeError("retry budget exhausted; retaining lease")

    def run_missions(self):
        self.wait(self.allocator.service_is_ready, 60, "C++ allocator service")
        for robot in self.actions:
            self.ready(robot)
        request = AllocateTasks.Request(
            robot_ids=list(self.actions),
            robot_cells=[cell(*p) for p in STARTS[: self.robots]],
            task_ids=list(self.actions),
            pickups=[cell(*p) for p in STARTS[: self.robots]],
            deliveries=[cell(*p) for p in DELIVERIES[: self.robots]],
            strategy="congestion",
            congestion_lambda=2.0,
        )
        response = self.future(
            self.allocator.call_async(request), 5, "C++ task allocation"
        )
        if not response.accepted or len(response.assignments) != self.robots:
            raise RuntimeError("allocation failed: " + response.error)
        assignments = sorted(response.assignments, key=lambda a: a.robot_id)
        self.event(
            "allocation",
            assignments=[
                {
                    "robot": a.robot_id,
                    "task": a.task_id,
                    "estimated_grid_steps": a.estimated_grid_steps,
                    "pickup_route": list(a.pickup_route),
                    "delivery_route": list(a.delivery_route),
                }
                for a in assignments
            ],
        )
        for assignment in assignments:
            robot, task = assignment.robot_id, assignment.task_id
            epoch = self.lease.acquire(robot)
            if epoch is None:
                raise RuntimeError("transit resource already owned")
            self.event("lease_acquired", robot=robot, generation=epoch)
            pickup = STARTS[task - 1]
            self.navigate(robot, pickup, "pickup")
            self.event("pickup_completed", robot=robot, task=task)
            side = -1 if pickup[0] < 0 else 1
            self.navigate(robot, (side * 1.4, 0.0), "entry")
            self.navigate(robot, (-side * 1.4, 0.0), "transit")
            self.navigate(robot, DELIVERIES[task - 1], "delivery")
            self.enabled = False
            # AMCL updates on movement. Fresh receipt AND recent source stamp required.
            x, y, received, stamp = self.pose[robot]
            fresh = time.monotonic() - received < 3 and self.now() - stamp < 2
            if not self.lease.release(
                robot, epoch, goal_succeeded=True, fresh_pose=fresh, x=x
            ):
                raise RuntimeError(
                    "cannot confirm clear transit resource; retaining lease"
                )
            self.completed.append(task)
            self.event(
                "lease_released",
                robot=robot,
                generation=epoch,
                amcl_x=x,
                amcl_y=y,
                pose_age=self.now() - stamp,
            )
            self.event(
                "task_completed", robot=robot, task=task, target=DELIVERIES[task - 1]
            )
        if self.fault != "none" and not self.injected:
            raise RuntimeError("fault was not injected")


def main():
    raise SystemExit(
        "Use scripts/sil/run_episode.py to retain logs, bags and validation artifacts."
    )


if __name__ == "__main__":
    main()
