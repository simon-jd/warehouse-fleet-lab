# Design and contracts

## Pipeline and time

Static map → fixed-seed offered tasks → bounded admission → allocator → reservation planner → synchronous executor → event/trace logs → independent evaluator. A task is **pickup followed by delivery**, not a single-target benchmark. One cardinal move or wait consumes one simulated tick. Robot radius, heading, kinematics, sensing, network delay and physical execution error are absent.

Wall-clock microseconds measure `allocate()` and `plan()` separately. Planning samples include A* setup/search but exclude reservation construction/commit; total process time includes loading, logging and Python verification. They are not physical control-cycle latency or an end-to-end real-time deadline. All policies use the same planner, motion model and offered task stream.

## Allocation

Nearest is deterministic greedy **total service distance** (`distance(robot,pickup) + distance(pickup,delivery)`), not pickup-only distance. Lower robot IDs act first; equal task costs prefer lower task ID. All methods see the oldest `2 * agents` pending task IDs. Batch distance uses a rectangular Hungarian solver, implemented here and checked against exhaustive small assignments. One dummy column per idle robot costs 1e9, statically unreachable assignments cost 1e12; thus finite reachable assignments are preferred. This choice is far above the maximum bounded-map/lambda cost.

Congestion uses the same distance matrix plus `lambda * congestion`. For each cell on a deterministic static two-leg shortest route, congestion sums the mean occupancy by **other** robots over the next W ticks, padding their committed path endpoints. Units: grid steps plus lambda grid-step-equivalents per expected occupied route cell. W=16 and lambda=2 are prespecified; lambda=0 must reproduce batch-distance traces. This estimate is deliberately simple: it ignores the candidate's arrival time and feedback within the batch. No guarantee of improved throughput follows.

BFS all-pairs distances are cached per static map, with a 64×64 input limit (at most 64 MiB distance storage). Dynamic closures do not change static distances; search handles them. The admitted outstanding-task cap, candidate cap, fleet cap (100), horizon cap (256), expansion cap and runner timeout are explicit. Historical task/event data is retained until run completion; this is a bounded-run simulator, not an indefinitely running task service.

## Planning and reservation ownership

Time-expanded A* searches cardinal steps and wait actions. Queue ties use `(f,time,cell)`, with static BFS distance as an admissible heuristic. Results distinguish `success`, static `unreachable`, `window_exhausted`, deterministic `expansion_budget` and optional nondeterministic wall-clock `deadline`. The default evaluation uses 2,000 expansions and a 64-tick window, with deadline disabled so replay does not depend on host scheduling. A failed search returns no partial plan.

A reservation table includes every robot's current position, committed trajectory and held endpoint through the horizon. Opposite edge swaps and same-tick vertex occupancy are forbidden. Each replacement is validated against other owners before commit. Failed replacement leaves old reservations intact. Existing paths persist between ticks; planning occurs when a robot needs a new leg or a fault epoch invalidates plans. Robot-ID priority is deterministic and not globally optimal or complete.

The executor checks legal motion, obstacles, closures, vertex conflicts and swaps using positions alone. A Python oracle separately checks recorded frames and task events. Reaching a goal cell on an intermediate detour does **not** complete service; service occurs only at the reserved path endpoint. This prevents early arrival from truncating a route into another robot's future reservation.

## State transitions

Every actual task/robot change emits a `task_state` or `robot_state` event.

| State | Allowed triggered next state |
|---|---|
| Task Pending | Assigned when selected by allocator |
| Task Assigned | InProgress at reserved pickup endpoint; Pending if unpicked robot becomes unavailable |
| Task InProgress | Completed at reserved delivery endpoint; cargo stays with paused robot |
| Task Completed | Terminal |
| Task Cancelled | Declared terminal vocabulary only; no cancellation API implemented |
| Robot Idle | Assigned by allocator; Unavailable on pause |
| Robot Assigned | Moving/Waiting on execution; Blocked on failed plan; Idle after zero-distance completion; Unavailable on pause |
| Robot Moving/Waiting | Assigned for a new successful leg; Blocked on replanning failure; Idle on completion; Unavailable on pause |
| Robot Blocked | Assigned after successful retry; Unavailable on pause |
| Robot Unavailable | Assigned if carrying a task, otherwise Idle, after fault clears |

Robot `Assigned` also means a newly accepted path for an in-progress task; it does not revert that task to Assigned. Trace `task_index` addresses the admitted-task array; event `task` uses stable offered IDs and may have gaps after admission rejection.

## Failure handling and concurrency boundary

Faults are injected only at a synchronous tick boundary. A pause or newly effective closure emits `plan_epoch_reset` and explicitly revokes **all** future paths, replacing them with current-position holds before replanning. This conservative global stop is valid only in the atomic-step simulator. It is not an asynchronous emergency-stop protocol or distributed consensus. The planner never silently edits another robot's plan during normal replacement.

An occupied closure is deferred until clear, never placed beneath a robot. A never-applied closure has `fault_applied=0` and null recovery. Overlapping faults on the same resource are rejected. Paused robots hold their cell; unpicked tasks return to Pending, loaded tasks retain ownership. Offered = admitted + rejected, admitted = completed + unfinished. No fault deletes a task.

No-progress means no pickup or completion for the configured interval while unfinished work exists; it is not a deadlock proof. Recovery means ticks from an **applied** fault clearing to the first fleet-wide task completion, not the affected robot's recovery or restoration of baseline throughput. Null is censored at cutoff. Compare fault/no-fault results separately; do not mix fault runs into allocator results.

The core is single-threaded. Concurrent robot actions are modeled synchronously; this is not a multithreaded/distributed deployment claim. The experiment runner bounds concurrent **isolated processes** to 1–4, defaults to one to reduce timing contention, and applies a subprocess timeout. It validates completion markers, state transitions, source/binary stability and paired workload identity before publishing successful evidence.

## Durable artifacts and limits

CLI output directories are create-exclusive. A rerun cannot overwrite a previous run. Exceptions leave diagnostics without COMPLETE; successful stream closes precede the marker. This is a local artifact-completion convention, not a transactional database or power-loss durability guarantee (there is no fsync protocol in the simulator). Gzip logs retain original SHA-256 values. The reader rejects incomplete/malformed traces; C++ `replay` validates spatial execution only, while the Python oracle additionally validates task transitions and applied closures.

The separate [ROS/Gazebo adapter](sil-design.md) now implements typed allocation, namespace/TF isolation, actual Nav2 actions, conservative feedback-confirmed ownership and independent physical-simulation checks. This discrete-core document does not transfer its synchronous reservation guarantees to the adapter. No cloud runtime, real robot, production uptime, ML training/inference or safety certification is claimed.
