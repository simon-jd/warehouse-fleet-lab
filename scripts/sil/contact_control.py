#!/usr/bin/env python3
"""Positive control for the contact oracle: intentionally drive ONE simulated chassis into a wall.

Runs a separate Gazebo world without Nav2/coordinator/gates. Never run on hardware.
Input is the generated world of a one-robot SIL episode. Failure to detect impact fails this test.
"""

import argparse
import json
import os
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
import rosbag2_py
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--generated", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if not Path("/.dockerenv").exists():
        raise SystemExit("requires an isolated Docker container")
    args.out.mkdir(parents=True, exist_ok=False)
    tree = ET.parse(args.generated / "world.sdf")
    world = tree.getroot().find("world")
    robots = [
        m for m in world.findall("model") if m.get("name", "").startswith("robot")
    ]
    if len(robots) != 1:
        raise SystemExit("positive control requires a one-robot world")
    robots[0].find("pose").text = "-.6 -2 .01 0 0 0"
    tree.write(args.out / "world.sdf", encoding="unicode")
    sim = Path(get_package_share_directory("nav2_minimal_tb3_sim"))
    env = os.environ | {
        "GZ_SIM_RESOURCE_PATH": str(sim / "models") + ":" + str(sim.parent),
        "LIBGL_ALWAYS_SOFTWARE": "1",
    }
    log = (args.out / "launch.log").open("w")
    processes = [
        subprocess.Popen(
            [
                "gz",
                "sim",
                "-r",
                "-s",
                "--headless-rendering",
                "--seed",
                "41",
                str(args.out / "world.sdf"),
            ],
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        ),
        subprocess.Popen(
            [
                "ros2",
                "run",
                "ros_gz_bridge",
                "parameter_bridge",
                "--ros-args",
                "-p",
                f"config_file:={args.generated / 'bridge.yaml'}",
            ],
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        ),
    ]
    rclpy.init()
    node = Node("contact_positive_control")
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(args.out / "bag"), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    state = {
        "clock": 0.0,
        "contacts": [],
        "odom_samples": 0,
        "first_x": None,
        "last_x": None,
    }
    for i, (topic, cls, typename) in enumerate(
        [
            ("/clock", Clock, "rosgraph_msgs/msg/Clock"),
            ("/robot1/contacts", Contacts, "ros_gz_interfaces/msg/Contacts"),
            ("/robot1/contacts_camera", Contacts, "ros_gz_interfaces/msg/Contacts"),
            ("/robot1/contacts_lidar", Contacts, "ros_gz_interfaces/msg/Contacts"),
            ("/robot1/ground_truth/odom", Odometry, "nav_msgs/msg/Odometry"),
        ]
    ):
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=i, name=topic, type=typename, serialization_format="cdr"
            )
        )

        def callback(msg, topic=topic):
            if topic == "/clock":
                state["clock"] = msg.clock.sec + msg.clock.nanosec / 1e9
            elif topic.rsplit("/", 1)[-1].startswith("contacts"):
                for contact in msg.contacts:
                    state["contacts"].append(
                        {
                            "sim_time": state["clock"],
                            "a": contact.collision1.name,
                            "b": contact.collision2.name,
                        }
                    )
            else:
                state["odom_samples"] += 1
                state["last_x"] = msg.pose.pose.position.x
                if state["first_x"] is None:
                    state["first_x"] = state["last_x"]
            writer.write(topic, serialize_message(msg), int(state["clock"] * 1e9))

        node.create_subscription(cls, topic, callback, qos_profile_sensor_data)
    publisher = node.create_publisher(Twist, "/robot1/cmd_vel", 10)
    end = time.monotonic() + 25
    try:
        while time.monotonic() < end and len(state["contacts"]) < 3:
            rclpy.spin_once(node, timeout_sec=0.05)
            command = Twist()
            command.linear.x = 0.3
            publisher.publish(command)
        state["passed"] = (
            len(state["contacts"]) >= 3
            and state["odom_samples"] > 20
            and any("wall_4" in c["a"] or "wall_4" in c["b"] for c in state["contacts"])
        )
    finally:
        publisher.publish(Twist())
        writer.close()
        node.destroy_node()
        rclpy.shutdown()
        for process in processes:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        log.close()
        (args.out / "result.json").write_text(json.dumps(state, indent=2) + "\n")
    print(json.dumps(state, indent=2))
    return 0 if state["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
