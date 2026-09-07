#!/usr/bin/env python3
"""Exercise bounded coordinator readiness polling against real deferred DDS replies."""

import argparse
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts/sil"))
from readiness import readiness_round


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    executable = str(
        Path(get_package_prefix("nav2_lifecycle_manager"))
        / "lib/nav2_lifecycle_manager/lifecycle_recovery_tests"
    )
    results = []
    for mode in ["first_get_lost", "get_silent", "manager_inactive"]:
        rclpy.init()
        node = Node("readiness_regression")
        with (args.out / (mode + ".log")).open("w") as log:
            process = subprocess.Popen(
                [executable, "server_" + mode], stdout=log, stderr=subprocess.STDOUT
            )
            try:
                state = node.create_client(GetState, "/target/get_state")
                manager = node.create_client(Trigger, "/target/is_active")
                assert state.wait_for_service(
                    timeout_sec=5
                ) and manager.wait_for_service(timeout_sec=5)
                queries = [
                    (state, GetState.Request(), lambda r: r.current_state.id == 1),
                    (manager, Trigger.Request(), lambda r: r.success),
                ]
                observed = []
                for index in range(3):
                    started = time.monotonic()
                    ready = readiness_round(node, queries, 0.25 if index == 0 else 1.0)
                    elapsed = time.monotonic() - started
                    assert ready == (mode == "first_get_lost" and index > 0)
                    assert elapsed < (0.75 if index == 0 else 1.5)
                    # Inspect pinned Jazzy client bookkeeping to prove timed-out
                    # futures do not accumulate across read-only polling rounds.
                    assert not state._pending_requests and not manager._pending_requests
                    observed.append({"ready": ready, "wall_seconds": elapsed})
                results.append({"case": mode, "passed": True, "rounds": observed})
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
                node.destroy_node()
                rclpy.shutdown()
    (args.out / "summary.json").write_text(
        json.dumps({"passed": True, "cases": results}, indent=2) + "\n"
    )
    print("Three real-DDS readiness retry/cleanup/inactive-manager cases passed.")


if __name__ == "__main__":
    main()
