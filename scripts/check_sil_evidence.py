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
                "archive_and_source_hashes": "matched",
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
