# Bounded lifecycle recovery — 2026-09-06 PDT / 2026-09-07 UTC

A missing ROS lifecycle response previously stalled fleet startup. The project now uses a small Nav2 1.3.12 overlay with finite deadlines and state reconciliation. It retains Nav2's lifecycle ordering, bond monitoring and controller-respawn logic. The coordinator requires both localization and navigation managers to report active before allowing motion.

## Diagnosis and engineering priority

The retained `qualification-installed-41/controller-restart/launch.log` contains `map_server/change_state` response timeout and `client will not receive response`. No mission or injected controller failure had started. The container uses `rmw_fastrtps_cpp` 8.4.4 and Fast DDS 2.14.6.

The [Fast DDS ROS adapter's response implementation](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/rmw_response.cpp) waits for the response reader to match the server writer. Request and response discovery can complete at different times. [Upstream issue #842](https://github.com/ros2/rmw_fastrtps/issues/842), opened August 12, 2025 and still open when checked September 7, 2026 UTC, describes a similar race. This identifies a plausible failure mechanism, not a complete reproduction of our original middleware trigger. No DDS packet trace was captured during that original failure.

The application hang is independently established: [Nav2 1.3.12's manager](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_lifecycle_manager/src/lifecycle_manager.cpp) calls the [service-client overload without a response deadline](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_util/include/nav2_util/service_client.hpp). The defect is the unbounded wait after an uncertain RPC outcome.

The [existing official JD sample](market-and-technology.md) motivates interface reliability, bounded work and observable recovery. Priorities were: (1) bound all lifecycle discovery/response waits, (2) reconcile actual state without replaying uncertain mutations, (3) exercise the installed manager with real DDS and a full Gazebo response-loss case. The DDS implementation and Nav2 base version remain fixed so this change can be attributed to the application patch. Switching middleware is an optional future comparison, not a required or claimed fix here.

## Recovery contract

* Default RPC budget: **1 wall second**, including discovery and response waiting. Default whole-transition budget: **15 wall seconds**. Both use a steady clock, are validated as finite positive parameters, and are independent of paused simulation time.
* Construction does not wait for an absent node. The manager remains able to answer its own services and report failure.
* Read the initial state; issue at most **one state-changing request** when the expected source state is observed. An already-reached target is an idempotent success. An unrelated state is failure.
* An explicit negative response is failure. A positive response alone is insufficient: confirm the actual target state.
* On response timeout, remove the pending client request and perform bounded read-only state reconciliation. A late response cannot complete a later request. Delayed state changes can still finish within the transition deadline.
* Seeing the source state after a timeout does **not** prove that the old request is absent from the server queue. Never resend it automatically. If the target cannot be confirmed, stop with an unresolved result and keep the manager inactive.
* The contract assumes one serialized lifecycle owner per node. It does not add distributed fencing, durable transactions, or cancellation of already-dispatched lifecycle callbacks.

The finite RPC deadline also handles slow callbacks: a timeout log need not imply a lost DDS packet. Logs distinguish response timeout, successful state reconciliation, rejection and unresolved outcomes.

Coordinator readiness also uses bounded **read-only** polling. A missed state/status response resets readiness stability instead of aborting a recovery immediately. Every completed or expired request is removed from client bookkeeping. Motion permission stays disabled throughout readiness checking, while the overall readiness deadline and existing transit ownership remain in force. Three additional real-DDS cases verify a lost first response followed by recovery, persistent silence and an inactive manager, including zero pending requests after each polling round.

## Validation design

Seventeen distinct real-DDS cases cover normal operation, executed requests with suppressed replies, requests never applied, late application before/after the deadline, late replies, explicit rejection, false success, missing state responses, an initial lost state response, absent services, an already-reached target and an invalid initial state. Four cases instantiate the patched manager itself and verify both its management response and active-status service. Test assertions count actual received mutations; Release builds do not compile them out.

A separate baseline probe uses the unmodified installed `nav2_util::LifecycleServiceClient` with the same deferred-response service fixture. Its nominal request completes; suppressing the reply after applying the transition leaves it blocked until the test harness kills that isolated probe after five seconds. This is a regression control, not a measured five-second natural timeout or a speedup claim.

The four manager scenarios are also repeated 25 times each in fresh sequential processes: **100 controlled manager-process trials**, not 100 distinct test cases or 100 full-fleet/Gazebo cold starts. These include deliberately failing startup with a bounded response; "passed" means the expected success/failure behavior was observed.

The additional three-robot Gazebo case routes only robot 3's `map_server/change_state` through a test proxy. It forwards to the real map server and suppresses the first successful configure response. The oracle requires one configure request, one activation request, a reconciliation log, zero sampled final commands before mission start, completed deliveries and the normal physical checks. This case is never eligible for the qualifier's clean-world startup retry.

## Reproduce and inspect

```bash
python3 scripts/sil/qualify.py --out runs/sil/new-lifecycle-qualification
```

The command builds a normal colcon installation, runs native ROS contract/recovery checks and controls, then executes all six SIL cases and the collision positive control. Native CTest and sanitizers are separate commands in [verification.md](verification.md). Hosted CI runs the eight allocator, 17 lifecycle and three readiness DDS cases, plus the original-hang control; the 100-trial campaign and physics episodes are local evidence.

Source: [reconciled client](../ros2_ws/src/nav2_lifecycle_manager/src/reconciled_lifecycle_client.cpp), [fault fixture](../ros2_ws/src/nav2_lifecycle_manager/test/lifecycle_recovery_tests.cpp), [real-map proxy](../ros2_ws/src/nav2_lifecycle_manager/test/lifecycle_reply_proxy.cpp), [patch provenance and ABI limits](../ros2_ws/src/nav2_lifecycle_manager/LOCAL-PATCH.md).

The overlay is qualified only in this pinned container with non-composed standalone managers. The manager's internal ABI changes; external code directly constructing it requires rebuilding. No middleware upgrade, physical hardware, production deployment or general fault-rate reduction is claimed. Fast DDS's underlying discovery race is not repaired by this patch.

## Measured final results

DDS fixtures used 0.15-second RPC / 1.2-second transition budgets to exercise deadline behavior efficiently; the full SIL runtime uses 1 / 15 seconds.

| Controlled DDS case | Result | Wall seconds | Received mutations |
|---|---|---:|---:|
| nominal | expected behavior passed | 0.000 | 1 |
| drop_after_apply | expected behavior passed | 0.151 | 1 |
| drop_before_apply | expected behavior passed | 1.200 | 1 |
| late_apply | expected behavior passed | 0.402 | 1 |
| apply_after_deadline | expected behavior passed | 1.200 | 1 |
| late_reply | expected behavior passed | 0.153 | 2 |
| rejected | expected behavior passed | 0.000 | 1 |
| false_success | expected behavior passed | 1.200 | 1 |
| get_silent | expected behavior passed | 1.200 | 0 |
| first_get_lost | expected behavior passed | 0.201 | 1 |
| absent | expected behavior passed | 1.200 | 0 |
| already_target | expected behavior passed | 0.000 | 0 |
| invalid_initial | expected behavior passed | 0.000 | 0 |
| manager_nominal | expected behavior passed | 0.002 | 2 |
| manager_drop_after_apply | expected behavior passed | 0.303 | 2 |
| manager_drop_before_apply | expected behavior passed | 1.202 | 1 |
| manager_absent | expected behavior passed | 1.202 | 0 |

Mutation counts span the entire case. `late_reply` and successful manager cases intentionally configure and then activate: two different transitions, each sent once. The `late_reply` interval above measures configuration; its separate activation and delayed-response check are additional assertions.

All 100 fresh manager-process trials matched their expected outcomes. The maximum recorded management-call interval was 1.204 seconds; this is a controlled local observation, not a production bound. [17-case results](evidence/sil/lifecycle-tests-summary.json), [100-trial results](evidence/sil/lifecycle-stress-summary.json), [unmodified-client baseline](evidence/sil/lifecycle-baseline-summary.json). Raw logs are inside the hash-indexed archives.

The final Gazebo response-loss case completed all three tasks, forwarded configuration once and observed 383 pre-mission zero commands. All six physics cases and the contact control passed; [full measurements](sil-validation.md).
