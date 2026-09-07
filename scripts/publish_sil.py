#!/usr/bin/env python3
"""Package an already-passed SIL suite; preserve exact raw evidence and source digests."""

import argparse
import hashlib
import json
import shutil
import tarfile
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("suite", type=Path)
    parser.add_argument("--out", type=Path, default=Path("docs/evidence/sil"))
    args = parser.parse_args()
    summary = json.loads((args.suite / "summary.json").read_text())
    assert (
        summary["passed"]
        and summary["source_unchanged_during_suite"]
        and (args.suite / "COMPLETE").exists()
    )
    assert not (args.out / "artifacts.json").exists(), (
        "Do not overwrite published evidence"
    )
    args.out.mkdir(parents=True, exist_ok=True)
    for name in [
        "summary.json",
        "environment.json",
        "protocol.json",
        "adapter-tests.log",
        "image-build.log",
        "COMPLETE",
    ]:
        shutil.copyfile(args.suite / name, args.out / name)
    archives = {}
    directories = [r["case"] for r in summary["cases"]] + ["contact-positive-control"]
    directories += [
        r["artifact_directory"] for r in summary.get("startup_failures", [])
    ]
    for name in directories:
        target = args.out / (name + ".tar.gz")
        with tarfile.open(target, "x:gz", compresslevel=9) as archive:
            for notice in Path("third_party").iterdir():
                if notice.is_file():
                    archive.add(notice, arcname=str(notice), recursive=False)
            for path in sorted((args.suite / name).rglob("*")):
                if path.is_file():
                    archive.add(
                        path, arcname=str(path.relative_to(args.suite)), recursive=False
                    )
        archives[target.name] = {
            "sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
            "bytes": target.stat().st_size,
        }
        if (args.suite / name / "verification.json").exists():
            shutil.copyfile(
                args.suite / name / "verification.json",
                args.out / (name + "-verification.json"),
            )
    manifest = {
        "schema_version": 1,
        "scope": "local Gazebo Harmonic SIL; not hardware/HIL",
        "archives": archives,
    }
    (args.out / "artifacts.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
