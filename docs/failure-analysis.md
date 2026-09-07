# Failure analysis

## Reproduced correctness bug: service before the reserved endpoint

Development map, four robots, seed 91, distance policy, lambda=0, 60 ticks. A* can visit a goal before its final endpoint, leave to respect another reservation, and return when it is safe to hold. The initial executor treated any visit as pickup/delivery, truncating a still-committed detour. At tick 7, robot 1 stopped at cell 54 while robot 2 held a future reservation there. The reservation constructor rejected the next planning cycle (`robot=2 other=1 cell=54 time=3`), so this defect was observable before a physical step was accepted.

Fix: process service only when the committed path contains the current endpoint alone. `goal_service_waits_for_reserved_endpoint`, the seed-91 CLI case, and repeated randomized simulations guard the boundary. A separate isolated-build mutation removes that check and reproduces failure; the fixed binary completes 21 tasks with zero independently checked conflicts. [Recorded before/after result](evidence/service-regression.json).

Reproduce without modifying the working tree:

```bash
python3 scripts/reproduce_service_regression.py
```

This is a correctness result, not an execution-speed improvement. The mutation intentionally rebuilds a buggy copy under a temporary directory.

## Expected algorithmic failure: conservative holds can stop progress

The priority planner is incomplete. A robot's parked endpoint can obstruct a narrow route; a loaded robot retains ownership while paused. Repeated window/budget failures can leave feasible joint rearrangements undiscovered. The executor may remain collision-free while work is unfinished. No-progress events report that condition without claiming a formal deadlock proof. The held-out cross-map results and failure counters are retained, including cases where congestion allocation performs worse.

Increasing fleet size can reduce throughput when stationary occupancy blocks a bottleneck. The run index and comparison plot expose this rather than hiding low-throughput seeds. A future separate experiment could evaluate parking policies, priority rotation or a complete MAPF solver; changing those together with allocation would confound the current comparison.

## Fault injection and recovery definition

The trace-derived demo uses narrow.map, 3 robots, seed 100, 160 ticks, congestion lambda=2, a 2,000-expansion limit and arrival every 2 ticks. Robot 0 pauses at tick 40 for 20 ticks; cell 252 closes at tick 100 for 20 ticks. Both faults were applied. Recovery to the first **fleet-wide** completion after clearing was 10 and 2 ticks respectively. It may be an unaffected robot that completes; this metric is not per-robot navigation recovery.

Both fault and paired no-fault runs offered/admitted 80 tasks, completed 18 and left 62 unfinished, with zero checked collisions. Therefore this sample shows preserved accounting and resumed fleet progress, **not** an improvement in throughput or a statistically established absence of fault cost. The independent workload hashes match. [Fault summary](evidence/recovery/fault-summary.json), [control](evidence/recovery/control-summary.json), [events](evidence/recovery/events.jsonl), [trace](evidence/recovery/trace.csv).

An occupied closure is deferred; a closure that never applies has no recovery sample. That condition is covered by a unit regression. Failures after cutoff are censored, not assigned a zero recovery time.

## Validation limitations

The first full matrix was exploratory and overlapped code hardening and sanitizer builds. It is excluded from final performance evidence. Final experiments use the frozen source/binary with one runner worker and record hashes; results still reflect a shared local host, not a dedicated performance machine or production environment. No significance claim follows from ten seeds.

LeakSanitizer initially failed inside the managed ptrace sandbox. The full final sanitizer suite was rerun outside that restriction with ASan, UBSan and leak checking enabled, and passed. The original failed XML is kept separately; it is not counted as a passing verification.
