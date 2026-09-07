# Warehouse Fleet Autonomy Lab

A C++20 planning simulator that asks: **does congestion-aware task allocation improve warehouse throughput when the planner and task stream stay fixed?** It models pickup→delivery tasks, concurrent grid motion and bounded reservation planning, with independent collision checks and reproducible experiments.

![Paired allocator comparison](docs/assets/comparison.png)

![Three-robot pause and closure replay](docs/assets/recovery.gif)

[Download the MP4 demo](docs/assets/recovery.mp4). The animation is generated from an actual discrete trace. This is a local simulation project; ROS 2/Gazebo integration and physical robots remain future work. Built with coding-agent assistance.

## Build and reproduce

CMake 3.20+, a C++20 compiler and Python 3.10+; no downloaded C++ or Python dependencies for the core, tests or benchmark.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build --output-on-failure
mkdir -p runs
./build/fleet simulate --map maps/narrow.map --agents 10 --seed 100 \
  --strategy congestion --lambda 2 --max-expansions 2000 --out runs/demo
./build/fleet replay --map maps/narrow.map --trace runs/demo/trace.csv
python3 benchmarks/run.py --out runs/evaluation
```

Choose a **new** output directory each time. Completed runs contain summary.json, events.jsonl, trace.csv and COMPLETE; an interrupted/failed run is not benchmark evidence. The batch runner compresses raw logs after independent verification. `--help` lists budgets, queue limits and fault inputs. The standard protocol is 360 runs: 3 maps × 3 fleet sizes × 10 seeds × (3 methods + lambda=0 ablation).

```bash
./build/fleet simulate --map maps/narrow.map --agents 3 --ticks 160 --seed 100 \
  --strategy congestion --lambda 2 --max-expansions 2000 \
  --faults configs/faults.txt --out runs/fault-demo
python3 scripts/check_evidence.py
```

Optional plot generation uses `requirements-tools.txt` and `scripts/render_results.py`. Example:

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements-tools.txt
.venv/bin/python scripts/render_results.py --results docs/evidence/held-out \
  --demo docs/evidence/recovery --map maps/narrow.map
```

## What is implemented

- Deterministic greedy total-service-distance allocation, rectangular Hungarian assignment, and a congestion penalty with a zero-lambda control.
- Time-expanded A*, endpoint holding, opposite-edge/vertex exclusion and validate-before-commit reservation updates.
- Queue/candidate/search budgets, typed planning outcomes, explicit fault epochs, task requeue/cargo retention and no-progress events.
- JSON metrics/events, trace replay, an independent C++ execution checker and Python artifact oracle; source/binary fingerprints and bounded subprocess experiments.
- CTest regressions, address/undefined/leak sanitizer verification, GitHub Actions and trace-derived recovery visualization.

[Evaluation and measured results](docs/evaluation.md) · [Architecture and state transitions](docs/design.md) · [Failure analysis](docs/failure-analysis.md) · [Market rationale](docs/market-and-technology.md) · [Upstream compatibility](docs/upstream.md) · [Verification](docs/verification.md) · [中文说明](README.zh-CN.md)

## Limits

The planner is prioritized and finite-horizon, so collision-free execution does not imply progress or completeness. Congestion scores are approximate; negative results are retained. There is no ROS adapter, continuous-space controller, hardware deployment, production service or ML model in this repository. The evidence describes synthetic workloads on local CPUs and, where separately linked, hosted CI.
