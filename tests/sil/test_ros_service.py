#!/usr/bin/env python3
"""Exercise real DDS calls against the compiled C++ adapter; no Gazebo required."""

import json
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from warehouse_fleet_ros.srv import AllocateTasks


def main():
    rclpy.init()
    node = Node("allocator_contract_test")
    results = []
    with tempfile.TemporaryDirectory(prefix="fleet-service-test-") as folder:
        grid = Path(folder) / "grid.map"
        grid.write_text("...\n.#.\n...\n")
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "warehouse_fleet_ros",
                "allocator_node",
                "--ros-args",
                "-p",
                f"grid_map:={grid}",
            ]
        )
        try:
            client = node.create_client(AllocateTasks, "/fleet/allocate")
            if not client.wait_for_service(timeout_sec=15):
                raise TimeoutError("allocator not available")
            good = {
                "robot_ids": [1],
                "robot_cells": [0],
                "task_ids": [1],
                "pickups": [0],
                "deliveries": [8],
                "strategy": "distance",
                "congestion_lambda": 2.0,
            }
            tests = [
                ("valid", {}, True),
                ("repeat_valid", {}, True),
                ("bad_dimensions", {"robot_cells": []}, False),
                (
                    "duplicate_robot",
                    {"robot_ids": [1, 1], "robot_cells": [0, 1]},
                    False,
                ),
                ("blocked_cell", {"robot_cells": [4]}, False),
                ("invalid_strategy", {"strategy": "missing"}, False),
                ("nonfinite_cost", {"congestion_lambda": float("nan")}, False),
                (
                    "oversized_tasks",
                    {
                        "task_ids": list(range(33)),
                        "pickups": [0] * 33,
                        "deliveries": [8] * 33,
                    },
                    False,
                ),
            ]
            previous = None
            for name, changes, expected in tests:
                future = client.call_async(AllocateTasks.Request(**(good | changes)))
                end = time.monotonic() + 5
                while not future.done() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
                if not future.done():
                    raise TimeoutError(name)
                response = future.result()
                assert response.accepted == expected, name
                if expected:
                    assert len(response.assignments) == 1
                    assignment = response.assignments[0]
                    assert assignment.robot_id == 1 and assignment.task_id == 1
                    route = list(assignment.delivery_route)
                    assert route[0] == 0 and route[-1] == 8 and 4 not in route
                    assert assignment.estimated_grid_steps == 4
                    if previous is not None:
                        assert route == previous
                    previous = route
                else:
                    assert response.error and not response.assignments
                results.append({"case": name, "passed": True})
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
    print(json.dumps({"passed": True, "cases": results}, indent=2))


if __name__ == "__main__":
    main()
