#!/usr/bin/env python3
"""Run isolated real-DDS lifecycle fault cases against the installed patched manager."""

import argparse
import json
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_prefix

CASES = [
    "nominal",
    "drop_after_apply",
    "drop_before_apply",
    "late_apply",
    "apply_after_deadline",
    "late_reply",
    "rejected",
    "false_success",
    "get_silent",
    "first_get_lost",
    "absent",
    "already_target",
    "invalid_initial",
    "manager_nominal",
    "manager_drop_after_apply",
    "manager_drop_before_apply",
    "manager_absent",
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--manager-only", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    assert 1 <= args.repeat <= 100
    executable = (
        Path(get_package_prefix("nav2_lifecycle_manager"))
        / "lib/nav2_lifecycle_manager/lifecycle_recovery_tests"
    )
    assert str(executable).startswith("/work/ros2_ws/install/"), executable
    results = []
    selected = [c for c in CASES if not args.manager_only or c.startswith("manager_")]
    for index, case in enumerate(selected * args.repeat):
        run = subprocess.run(
            [str(executable), case, "--ros-args", "--log-level", "warn"],
            text=True,
            capture_output=True,
            timeout=20,
            check=False,
        )
        log_name = f"{index:03}-{case}.log"
        (args.out / log_name).write_text(run.stdout + run.stderr)
        rows = [
            json.loads(line.removeprefix("FLEET_TEST "))
            for line in run.stdout.splitlines()
            if line.startswith("FLEET_TEST ")
        ]
        passed = run.returncode == 0 and len(rows) == 1 and rows[0]["passed"]
        results.append(
            {"case": case, "passed": passed, "observations": rows, "log": log_name}
        )
        print(f"{case}: {'PASS' if passed else 'FAIL'}", flush=True)
    summary = {"passed": all(r["passed"] for r in results), "cases": results}
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
