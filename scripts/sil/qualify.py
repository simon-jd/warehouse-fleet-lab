#!/usr/bin/env python3
"""Host entry point: build and run the fixed SIL qualification suite, retaining failures."""

import argparse
import hashlib
import json
import os
import platform
import subprocess
import uuid
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
IMAGE = "warehouse-fleet:jazzy-sil"
CASES = [
    ("single", 1, "none"),
    ("three", 3, "none"),
    ("scan-delay", 3, "scan_delay"),
    ("actuator-hold", 3, "actuator_hold"),
    ("controller-restart", 3, "nav_restart"),
]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def startup_retryable(episode):
    """A fresh simulator may retry readiness only before any motion/mission ownership."""
    try:
        result = json.loads((episode / "result.json").read_text())
        events = [
            json.loads(line)
            for line in (episode / "events.jsonl").read_text().splitlines()
        ]
        telemetry = [
            json.loads(line)
            for line in (episode / "telemetry.jsonl").read_text().splitlines()
        ]
        return (
            result.get("passed") is False
            and "mission_start_sim" not in result
            and "retained_lease_owner" in result
            and result.get("retained_lease_owner") is None
            and isinstance(result.get("error"), str)
            and result["error"].startswith("TimeoutError: robot")
            and all(e["kind"] == "robot_ready" for e in events)
            and any(t["topic"].endswith("/cmd_vel") for t in telemetry)
            and all(
                abs(t["v"]) + abs(t["w"]) == 0
                for t in telemetry
                if t["topic"].endswith("/cmd_vel")
            )
        )
    except (OSError, ValueError, KeyError, TypeError):
        return False


def source_hashes():
    files = [
        *ROOT.glob("scripts/sil/*.py"),
        *ROOT.glob("tests/sil/*.py"),
        *ROOT.glob("ros2_ws/src/warehouse_fleet_ros/**/*"),
        ROOT / "ros2_ws/src/warehouse_fleet_ros/CMakeLists.txt",
        ROOT / "ros2/Dockerfile",
        ROOT / "include/fleet/fleet.hpp",
        *(
            ROOT / "src" / name
            for name in ["map.cpp", "task_allocator.cpp", "reservation_planner.cpp"]
        ),
    ]
    return {
        str(p.relative_to(ROOT)): digest(p)
        for p in sorted(set(files))
        if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"
    }


def container(command, args, log):
    name = "wfal-qualification-" + uuid.uuid4().hex[:12]
    docker = [
        "docker",
        "run",
        "--rm",
        "--name",
        name,
        "--network",
        "none",
        "--shm-size=1g",
        "--ulimit",
        "core=0",
        "-e",
        "ROS_DOMAIN_ID=81",
        "-e",
        "GZ_PARTITION=" + name,
        "-e",
        "OMP_NUM_THREADS=2",
        "--mount",
        f"type=bind,source={ROOT},target=/work",
        IMAGE,
        "bash",
        "-lc",
        "source /opt/ros/jazzy/setup.bash && " + command,
        "sil",
        *args,
    ]
    try:
        with log.open("w") as stream:
            return subprocess.run(
                docker,
                stdout=stream,
                stderr=subprocess.STDOUT,
                timeout=960,
                check=False,
            ).returncode
    except subprocess.TimeoutExpired:
        return 124
    finally:
        subprocess.run(
            ["docker", "rm", "-f", name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=41)
    args = parser.parse_args()
    out = args.out.resolve()
    relative = out.relative_to(ROOT)
    out.mkdir(parents=True, exist_ok=False)
    inside = "/work/" + str(relative)
    start_hashes = source_hashes()
    protocol = {
        "seed": args.seed,
        "cases": CASES,
        "execution": "sequential, isolated Docker PID/network/Gazebo partitions",
        "fault_duration_sim_seconds": 4,
        "scan_delay_sim_seconds": 1,
        "fault_trigger": "robot1 entry action +2 simulation seconds",
        "goal_wall_timeout_seconds": 180,
        "goal_attempts": 3,
        "episode_wall_timeout_seconds": 900,
        "startup_attempts_per_case": 2,
        "startup_retry_condition": "Readiness timeout before mission start; no ownership and zero recorded final velocity. Entire isolated container/world is recreated; failed attempt retained.",
        "min_robot_center_separation_m": 0.48,
        "min_wall_center_clearance_m": 0.24,
        "max_delivery_error_m": 0.45,
        "max_truth_sample_gap_s": 0.15,
        "limitations": [
            "One seed per case; qualification examples, not failure-rate estimates.",
            "SIL, not real hardware or hardware-in-the-loop.",
            "All Nav2 stacks run concurrently; the transit lease serializes complete missions.",
            "Physics/DDS/MPPI timing is nondeterministic; fixed seed does not imply byte-identical results.",
        ],
    }
    (out / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    print("Building pinned dependency image and C++ ROS adapter...", flush=True)
    with (out / "image-build.log").open("w") as stream:
        subprocess.run(
            ["docker", "build", "-t", IMAGE, "-f", "ros2/Dockerfile", "."],
            cwd=ROOT,
            stdout=stream,
            stderr=subprocess.STDOUT,
            check=True,
            timeout=900,
        )
    code = container(
        "cd /work/ros2_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash && python3 /work/tests/sil/test_ros_service.py",
        [],
        out / "adapter-tests.log",
    )
    if code:
        raise SystemExit("Adapter build/service tests failed; see retained log")
    packages = subprocess.check_output(
        [
            "docker",
            "run",
            "--rm",
            "--network",
            "none",
            IMAGE,
            "dpkg-query",
            "-W",
            "ros-jazzy-navigation2",
            "ros-jazzy-nav2-bringup",
            "ros-jazzy-nav2-minimal-tb3-sim",
            "ros-jazzy-ros-gz",
            "ros-jazzy-gz-sim-vendor",
            "ros-jazzy-rosbag2-storage-mcap",
        ],
        text=True,
    )
    environment = {
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "host_platform": platform.platform(),
        "logical_cpus": os.cpu_count(),
        "cpu_model": next(
            (
                s.split(":", 1)[1].strip()
                for s in Path("/proc/cpuinfo").read_text().splitlines()
                if s.startswith("model name")
            ),
            "unknown",
        ),
        "memory_kib": int(Path("/proc/meminfo").read_text().splitlines()[0].split()[1]),
        "image_id": subprocess.check_output(
            ["docker", "image", "inspect", "--format", "{{.Id}}", IMAGE], text=True
        ).strip(),
        "packages": packages,
        "rendering": "headless Ogre2, Mesa software rendering; no GPU passthrough",
        "source_sha256": start_hashes,
        "allocator_binary_sha256": digest(
            ROOT
            / "ros2_ws/install/warehouse_fleet_ros/lib/warehouse_fleet_ros/allocator_node"
        ),
    }
    (out / "environment.json").write_text(json.dumps(environment, indent=2) + "\n")
    results = []
    startup_failures = []
    for name, robots, fault in CASES:
        print(f"Running {name}...", flush=True)
        run_code = container(
            'source /work/ros2_ws/install/setup.bash && python3 /work/scripts/sil/run_episode.py "$@"',
            [
                "--out",
                inside + "/" + name,
                "--robots",
                str(robots),
                "--fault",
                fault,
                "--seed",
                str(args.seed),
            ],
            out / (name + ".log"),
        )
        if run_code and startup_retryable(out / name):
            failed_name = name + "-startup-1"
            (out / name).rename(out / failed_name)
            (out / (name + ".log")).rename(out / (failed_name + ".log"))
            startup_failures.append(
                {
                    "case": name,
                    "attempt": 1,
                    "artifact_directory": failed_name,
                    "classification": "pre-mission readiness timeout; no motion; not a completed fault scenario",
                }
            )
            print(
                f"{name}: retaining startup timeout; one bounded fresh-world retry",
                flush=True,
            )
            run_code = container(
                'source /work/ros2_ws/install/setup.bash && python3 /work/scripts/sil/run_episode.py "$@"',
                [
                    "--out",
                    inside + "/" + name,
                    "--robots",
                    str(robots),
                    "--fault",
                    fault,
                    "--seed",
                    str(args.seed),
                ],
                out / (name + ".log"),
            )
        verify_code = container(
            'python3 /work/scripts/sil/verify.py "$@"',
            [inside + "/" + name],
            out / (name + "-verify.log"),
        )
        report = out / name / "verification.json"
        result = (
            json.loads(report.read_text())
            if report.exists()
            else {"passed": False, "errors": ["verification did not finish"]}
        )
        result.update(case=name, runner_exit=run_code, verifier_exit=verify_code)
        result["passed"] = result["passed"] and run_code == 0 and verify_code == 0
        results.append(result)
        (out / "summary.json").write_text(
            json.dumps(
                {
                    "passed": False,
                    "cases": results,
                    "startup_failures": startup_failures,
                },
                indent=2,
            )
            + "\n"
        )
        print(f"{name}: {'PASS' if result['passed'] else 'FAIL'}", flush=True)
    code = container(
        'python3 /work/scripts/sil/contact_control.py "$@"',
        [
            "--generated",
            inside + "/single/generated",
            "--out",
            inside + "/contact-positive-control",
        ],
        out / "contact-control.log",
    )
    contact = (
        json.loads((out / "contact-positive-control/result.json").read_text())
        if (out / "contact-positive-control/result.json").exists()
        else {"passed": False}
    )
    unchanged = source_hashes() == start_hashes
    summary = {
        "passed": all(r["passed"] for r in results)
        and code == 0
        and contact["passed"]
        and unchanged,
        "cases": results,
        "startup_failures": startup_failures,
        "contact_positive_control_passed": contact["passed"],
        "source_unchanged_during_suite": unchanged,
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    if summary["passed"]:
        (out / "COMPLETE").write_text(datetime.now(timezone.utc).isoformat() + "\n")
    print(json.dumps(summary, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
