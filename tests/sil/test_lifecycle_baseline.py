#!/usr/bin/env python3
"""Controlled regression reproduction: unmodified Nav2 response wait hangs."""

import argparse
import json
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_prefix


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
    for case in ["baseline_nominal", "baseline_drop_after_apply"]:
        try:
            run = subprocess.run(
                [executable, case],
                capture_output=True,
                text=True,
                timeout=5,
                check=False,
            )
            text = run.stdout + run.stderr
            timed_out = False
            returncode = run.returncode
        except subprocess.TimeoutExpired as exc:
            text = (exc.stdout or b"").decode() + (exc.stderr or b"").decode()
            timed_out = True
            returncode = None
        (args.out / (case + ".log")).write_text(text)
        mutations = [
            json.loads(line.removeprefix("FLEET_MUTATION "))
            for line in text.splitlines()
            if line.startswith("FLEET_MUTATION ")
        ]
        assert mutations == [{"transition": 1, "state": 2}], (case, text)
        assert timed_out == (case == "baseline_drop_after_apply"), (case, text)
        if not timed_out:
            assert returncode == 0, text
        results.append(
            {
                "case": case,
                "observed_timeout": timed_out,
                "limit_seconds": 5,
                "mutations": mutations,
                "expected_behavior_observed": True,
            }
        )
    (args.out / "summary.json").write_text(
        json.dumps({"passed": True, "cases": results}, indent=2) + "\n"
    )
    print("Original Nav2 missing-response hang reproduced with executed transition.")


if __name__ == "__main__":
    main()
