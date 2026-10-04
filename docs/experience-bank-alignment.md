# Experience Bank implementation map

Result provenance: owner-confirmed separate cloud-hosted test results. The README reproduces the current selected bullets. Commands below exercise this checkout; new outcomes must be recorded separately from those supplied results.

| Result | Implementation and regression evidence | Reproduce | Scope / source difference |
|---|---|---|---|
| 1 | [src](../src) · [include](../include) · [tests/test_core.cpp](../tests/test_core.cpp) | `cmake -S . -B build && cmake --build build -j4 && ctest --test-dir build --output-on-failure` | Discrete planning has bounded expansions and reservations. |
| 2 | [benchmarks/run.py](../benchmarks/run.py) · [tests/test_oracle.py](../tests/test_oracle.py) | `python3 benchmarks/run.py --config configs/experience-bank.json --out runs/new-evaluation` | Independent oracle counts waits and reverse-edge conflicts; no continuous safety inference. |
| 3 | [tests/sil/test_ros_service.py](../tests/sil/test_ros_service.py) · [tests/sil/test_lifecycle_recovery.py](../tests/sil/test_lifecycle_recovery.py) · [tests/sil/test_readiness.py](../tests/sil/test_readiness.py) | `python3 scripts/sil/qualify.py --out runs/new-sil` | Native, DDS and repeated executions are separate counts. |
| 4 | [scripts/sil/run_episode.py](../scripts/sil/run_episode.py) · [scripts/sil/verify.py](../scripts/sil/verify.py) | `python3 scripts/sil/qualify.py --out runs/new-sil` | At most three robots in SIL; fixed scenario distances are observations. |
| 5 | [scripts/sil/protocol.py](../scripts/sil/protocol.py) · [scripts/sil/command_gate.py](../scripts/sil/command_gate.py) · [tests/sil/test_protocol.py](../tests/sil/test_protocol.py) | `ctest --test-dir build --output-on-failure` | Unknown transition ownership and expiring motion permissions do not certify braking. |

## Measurement boundaries

- 50 robots exists only in the discrete simulation; SIL runs at most 3 robots, and the two scales must never be mixed.
- A mean of 0 discrete vertex/reverse-edge conflicts is not continuous-space safety: the grid check covers only the defined discrete conflict model, while SIL is also affected by dynamics, perception and sampling, and there is no real-vehicle proof.
- Finite-priority planning can deadlock or fail to complete all tasks; no optimality or completeness claim is made.
- 3,240,000 robot steps are derived as (10+30+50) × 3 × 10 × 4 × 300 and count waiting as steps; they are not sensor-message counts.
- 16/16 final-state completion and the 1.08 m minimum center distance come from fixed-seed-41 local scenarios; the per-robot 0.22 m circular approximation covers center distance only and cannot represent body protrusion, discrete-sampling missed collisions or hardware braking.
- 65 distinct cases = 37 native + 8 allocation DDS + 17 lifecycle DDS + 3 readiness; 4 manager scenarios × 25 repetitions = 100 executions do not increase the distinct-case count.
- No physical hardware, cloud or production deployment; do not extrapolate real-vehicle safety or production throughput.

## Local verification

See `docs/alignment-verification.json` for commands and outcomes from this checkout. Supplied cloud numbers, historical checked-in artifacts and new local checks are separate evidence sets. A skipped dependency test is not a pass.
