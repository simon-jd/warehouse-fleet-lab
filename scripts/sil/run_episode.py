#!/usr/bin/env python3
"""Run one bounded, isolated physical-simulation episode and retain raw MCAP evidence."""

import argparse
import json
import os
import signal
import subprocess
import time
import traceback
from collections import Counter
from pathlib import Path

import rclpy
import rosbag2_py
from coordinator import Coordinator
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.serialization import serialize_message
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


class Recorder:
    def __init__(self, node, out, count):
        self.node = node
        self.counts = Counter()
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=str(out / "bag"), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
        self.stream = (out / "telemetry.jsonl").open("w")
        self.topics = []
        self.subscribe(
            "/clock", Clock, "rosgraph_msgs/msg/Clock", qos_profile_sensor_data
        )
        for r in range(1, count + 1):
            for name, cls, typename, qos in [
                (
                    "ground_truth/odom",
                    Odometry,
                    "nav_msgs/msg/Odometry",
                    qos_profile_sensor_data,
                ),
                ("odom", Odometry, "nav_msgs/msg/Odometry", qos_profile_sensor_data),
                (
                    "amcl_pose",
                    PoseWithCovarianceStamped,
                    "geometry_msgs/msg/PoseWithCovarianceStamped",
                    QoSProfile(
                        depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    ),
                ),
                (
                    "scan_raw",
                    LaserScan,
                    "sensor_msgs/msg/LaserScan",
                    qos_profile_sensor_data,
                ),
                (
                    "scan",
                    LaserScan,
                    "sensor_msgs/msg/LaserScan",
                    qos_profile_sensor_data,
                ),
                ("imu", Imu, "sensor_msgs/msg/Imu", qos_profile_sensor_data),
                ("cmd_vel_nav", Twist, "geometry_msgs/msg/Twist", 10),
                ("cmd_vel", Twist, "geometry_msgs/msg/Twist", 10),
                (
                    "contacts_camera",
                    Contacts,
                    "ros_gz_interfaces/msg/Contacts",
                    qos_profile_sensor_data,
                ),
                (
                    "contacts_lidar",
                    Contacts,
                    "ros_gz_interfaces/msg/Contacts",
                    qos_profile_sensor_data,
                ),
                (
                    "contacts",
                    Contacts,
                    "ros_gz_interfaces/msg/Contacts",
                    qos_profile_sensor_data,
                ),
                ("tf", TFMessage, "tf2_msgs/msg/TFMessage", qos_profile_sensor_data),
                (
                    "tf_static",
                    TFMessage,
                    "tf2_msgs/msg/TFMessage",
                    QoSProfile(
                        depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    ),
                ),
                ("gate/events", String, "std_msgs/msg/String", 100),
                ("permit", String, "std_msgs/msg/String", 10),
                ("inject_fault", String, "std_msgs/msg/String", 10),
            ]:
                self.subscribe(f"/robot{r}/{name}", cls, typename, qos)

    def subscribe(self, topic, cls, typename, qos):
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=len(self.topics),
                name=topic,
                type=typename,
                serialization_format="cdr",
            )
        )
        self.topics.append(topic)
        self.node.create_subscription(
            cls, topic, lambda msg: self.record(topic, msg), qos
        )

    def record(self, topic, msg):
        now = self.node.now()
        self.counts[topic] += 1
        self.writer.write(topic, serialize_message(msg), int(now * 1e9))
        # Small portable projection for review; verify.py independently reads the MCAP.
        data = {"topic": topic, "time": now}
        if topic.endswith("/ground_truth/odom"):
            data.update(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y)
        elif topic.endswith("/cmd_vel"):
            data.update(v=msg.linear.x, w=msg.angular.z)
        elif topic.endswith("/gate/events"):
            data.update(event=json.loads(msg.data))
        else:
            return
        self.stream.write(json.dumps(data, allow_nan=False) + "\n")

    def graph(self):
        result = {}
        for topic in self.topics:
            result[topic] = {
                "publishers": sorted(
                    {
                        i.node_namespace.rstrip("/") + "/" + i.node_name
                        for i in self.node.get_publishers_info_by_topic(topic)
                    }
                ),
                "subscribers": sorted(
                    {
                        i.node_namespace.rstrip("/") + "/" + i.node_name
                        for i in self.node.get_subscriptions_info_by_topic(topic)
                    }
                ),
            }
        return result

    def close(self):
        self.stream.close()
        self.writer.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--robots", type=int, choices=[1, 3], required=True)
    parser.add_argument(
        "--fault",
        choices=[
            "none",
            "scan_delay",
            "actuator_hold",
            "nav_restart",
            "lifecycle_reply_loss",
        ],
        default="none",
    )
    parser.add_argument("--seed", type=int, default=41)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    events = (out / "events.jsonl").open("w")

    def event_sink(data):
        events.write(json.dumps(data, allow_nan=False) + "\n")
        events.flush()
        print(json.dumps(data), flush=True)

    launch_log = (out / "launch.log").open("w")
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "warehouse_fleet_ros",
            "fleet.launch.py",
            f"robots:={args.robots}",
            f"run_dir:={out / 'generated'}",
            f"seed:={args.seed}",
            "lifecycle_fault:="
            + ("reply_loss" if args.fault == "lifecycle_reply_loss" else "none"),
        ],
        stdout=launch_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    started = time.monotonic()
    result = {
        "robots": args.robots,
        "fault": args.fault,
        "seed": args.seed,
        "scope": "local Gazebo Harmonic software-in-the-loop",
        "passed": False,
    }
    rclpy.init()
    node = Coordinator(
        args.robots,
        "none" if args.fault == "lifecycle_reply_loss" else args.fault,
        event_sink,
    )
    recorder = Recorder(node, out, args.robots)
    try:

        def deadline(signum, frame):
            raise TimeoutError("900 second episode wall deadline")

        signal.signal(signal.SIGALRM, deadline)
        signal.alarm(900)
        for robot in range(1, args.robots + 1):
            node.ready(robot, 120)
        graph = recorder.graph()
        (out / "graph.json").write_text(json.dumps(graph, indent=2) + "\n")
        for robot in range(1, args.robots + 1):
            pubs = graph[f"/robot{robot}/cmd_vel"]["publishers"]
            if pubs != [f"/robot{robot}/command_gate"]:
                raise RuntimeError(f"final velocity bypass: {pubs}")
        result["mission_start_sim"] = node.now()
        node.run_missions()
        result["mission_end_sim"] = node.now()
        result["completed_tasks"] = node.completed
        result["passed"] = True
    except Exception as exc:  # noqa: BLE001 - retain artifacts for every episode failure
        result["error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        signal.alarm(0)
        node.enabled = False
        end = time.monotonic() + 1
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.03)
        result["wall_seconds"] = time.monotonic() - started
        result["topic_counts"] = dict(recorder.counts)
        result["retained_lease_owner"] = node.lease.owner
        (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        recorder.close()
        events.close()
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        launch_log.close()
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
