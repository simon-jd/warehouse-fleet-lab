#!/usr/bin/env python3
"""Validate published archive/source hashes and independently re-read all raw MCAP bags.

Run in the qualified ROS Docker image. Extracts only into a temporary directory.
"""

import argparse
import hashlib
import json
import sys
import tarfile
import tempfile
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/sil"))
from verify import verify


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence", type=Path, default=ROOT / "docs/evidence/sil")
    args = parser.parse_args()
    manifest = json.loads((args.evidence / "artifacts.json").read_text())
    environment = json.loads((args.evidence / "environment.json").read_text())
    summary = json.loads((args.evidence / "summary.json").read_text())
    assert summary["passed"] and (args.evidence / "COMPLETE").exists()
    for name, digest in environment["source_sha256"].items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest, (
            "source changed: " + name
        )
    with tempfile.TemporaryDirectory(prefix="fleet-sil-replay-") as folder:
        temp = Path(folder)
        for name, info in manifest["archives"].items():
            path = args.evidence / name
            assert path.stat().st_size == info["bytes"]
            assert hashlib.sha256(path.read_bytes()).hexdigest() == info["sha256"], name
            with tarfile.open(path) as archive:
                archive.extractall(temp, filter="data")
        for expected in summary["cases"]:
            actual = json.loads(json.dumps(verify(temp / expected["case"])))
            assert actual["passed"], actual
            assert actual == {
                k: v
                for k, v in expected.items()
                if k not in ["case", "runner_exit", "verifier_exit"]
            }, expected["case"]
        for folder, expected_count in [
            ("lifecycle-tests", 17),
            ("lifecycle-stress", 100),
        ]:
            test_summary = json.loads((temp / folder / "summary.json").read_text())
            assert (
                test_summary["passed"] and len(test_summary["cases"]) == expected_count
            )
            for case in test_summary["cases"]:
                log = (temp / folder / case["log"]).read_text()
                rows = [
                    json.loads(line.removeprefix("FLEET_TEST "))
                    for line in log.splitlines()
                    if line.startswith("FLEET_TEST ")
                ]
                assert (
                    case["passed"] and rows == case["observations"] and len(rows) == 1
                )
                assert rows[0]["passed"] and rows[0]["case"] == case["case"]
        baseline = json.loads((temp / "lifecycle-baseline/summary.json").read_text())
        readiness = json.loads((temp / "readiness-tests/summary.json").read_text())
        assert readiness["passed"] and len(readiness["cases"]) == 3
        for case in readiness["cases"]:
            assert case["passed"] and len(case["rounds"]) == 3
            expected = (
                [False, True, True] if case["case"] == "first_get_lost" else [False] * 3
            )
            assert [r["ready"] for r in case["rounds"]] == expected
            assert all(
                r["wall_seconds"] < (0.75 if i == 0 else 1.5)
                for i, r in enumerate(case["rounds"])
            )
        assert baseline["passed"] and len(baseline["cases"]) == 2
        for case in baseline["cases"]:
            log = (temp / "lifecycle-baseline" / (case["case"] + ".log")).read_text()
            mutations = [
                json.loads(line.removeprefix("FLEET_MUTATION "))
                for line in log.splitlines()
                if line.startswith("FLEET_MUTATION ")
            ]
            assert mutations == case["mutations"] == [{"transition": 1, "state": 2}]
            assert case["observed_timeout"] == (
                case["case"] == "baseline_drop_after_apply"
            )
        for failure in summary.get("startup_failures", []):
            episode = temp / failure["artifact_directory"]
            receipt = json.loads((episode / "result.json").read_text())
            assert (
                receipt["passed"] is False
                and "mission_start_sim" not in receipt
                and receipt["retained_lease_owner"] is None
            )
            events = [
                json.loads(line)
                for line in (episode / "events.jsonl").read_text().splitlines()
            ]
            assert all(e["kind"] == "robot_ready" for e in events)
            failed_reader = rosbag2_py.SequentialReader()
            failed_reader.open(
                rosbag2_py.StorageOptions(uri=str(episode / "bag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""),
            )
            twist_type = get_message("geometry_msgs/msg/Twist")
            checked = 0
            while failed_reader.has_next():
                topic, data, _ = failed_reader.read_next()
                if topic.endswith("/cmd_vel"):
                    command = deserialize_message(data, twist_type)
                    assert abs(command.linear.x) + abs(command.angular.z) == 0
                    checked += 1
            assert checked > 0
        # Validate the positive control from serialized ROS observations, not its pass flag.
        control = temp / "contact-positive-control"
        receipt = json.loads((control / "result.json").read_text())
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(control / "bag"), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
        types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
        contacts, odometry = [], []
        while reader.has_next():
            topic, data, _stamp = reader.read_next()
            msg = deserialize_message(data, types[topic])
            if topic.rsplit("/", 1)[-1].startswith("contacts"):
                contacts.extend(
                    (c.collision1.name, c.collision2.name) for c in msg.contacts
                )
            elif topic.endswith("/ground_truth/odom"):
                odometry.append(msg.pose.pose.position.x)
        assert len(contacts) >= 3 and len(contacts) == len(receipt["contacts"])
        assert any("wall_4" in a or "wall_4" in b for a, b in contacts)
        assert len(odometry) == receipt["odom_samples"] and len(odometry) > 20
        assert odometry[0] == receipt["first_x"] and odometry[-1] == receipt["last_x"]
    print(
        json.dumps(
            {
                "passed": True,
                "verified_episodes": len(summary["cases"]),
                "positive_control_replayed": True,
                "lifecycle_regression_cases_checked": 17,
                "readiness_retry_cases_checked": 3,
                "lifecycle_manager_process_trials_checked": 100,
                "original_nav2_hang_control_checked": True,
                "retained_startup_failures_replayed": len(
                    summary.get("startup_failures", [])
                ),
                "archive_and_source_hashes": "matched",
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
