# Local Gazebo SIL qualification — 2026-09-06 PDT

**All five frozen cases passed**, with 13 completed pickup/delivery tasks and a separate deliberate-contact positive control. These are software-in-the-loop examples using actual ROS 2/Nav2 processes and Gazebo physics. They are not real hardware, hardware-in-the-loop, production operation or a statistical reliability study.

| Case, seed 41 | Robots / completed tasks | Mission simulation seconds | Whole episode wall seconds | Minimum observed pair distance | Maximum delivery position error |
|---|---:|---:|---:|---:|---:|
| Single robot | 1 / 1 | 26.916 | 34.249 | n/a | 0.172 m |
| Three robots, nominal | 3 / 3 | 81.624 | 90.389 | 1.158 m | 0.205 m |
| One-second lidar delay for four seconds | 3 / 3 | 83.457 | 92.511 | 1.138 m | 0.229 m |
| Four-second actuator command hold | 3 / 3 | 82.761 | 91.339 | 1.132 m | 0.244 m |
| Controller killed and respawned | 3 / 3 | 92.571 | 101.429 | 1.085 m | 0.252 m |

Numbers come from the [independent summary](evidence/sil/summary.json), not an animation or navigation success flag alone. The suite recorded **570,171 ROS messages**, including **53,094 ground-truth position observations**. It detected zero chassis/camera/lidar contacts and zero non-owner nonzero commands in the five normal/fault missions. The smallest sampled center-to-wall distance was 0.714 m against a 0.24 m footprint bound; the largest observed ground-truth sample gap was 0.042 s against the 0.15 s threshold. These finite sampled checks do not prove continuous collision avoidance.

![Recorded Gazebo controller-restart replay](assets/gazebo-recovery.gif)

The animation replays recorded Gazebo world-frame positions and lease events, accelerated for review. It is not simulator camera footage. [Static frame](assets/gazebo-recovery.png), [rendering script](../scripts/render_sil.py).

## Fault evidence

The lidar-delay case measured a maximum processed-message source age of **1.038 s**. The active goal was canceled with acknowledgment, returned canceled, and was retried under the same lease. There were 60 sampled zero final commands during the settled fault interval; maximum observed displacement in that interval was 1.4e-06 m. The gate retained the original scan timestamps; delayed data did not appear fresh on arrival.

The actuator-hold case measured a **4.029 s** injection interval and 60 sampled zero commands in its settled portion. Maximum settled displacement was **1.4e-06 m**. This is an artificial final-command hold, not a calibrated motor failure or a physical obstacle test. The drift is a simulator observation, not achievable hardware precision.

The restart case killed robot 1's actual `controller_server` PID 58; the recorded launch log contains the SIGKILL exit and replacement PID 1148. There were 30 zero-command observations between 0.5 and 2 simulation seconds after the injection. The coordinator retained ownership, waited for lifecycle activation, retried and completed all three missions. This tests a Nav2 controller restart; it does not test durable recovery after coordinator death.

The positive control intentionally drove a separate simulated robot into `wall_4`, producing three recorded `camera_link` contact observations. It is excluded from the five cases' zero-contact count. [Development findings](evidence/sil/development-runs.json) retain the earlier failed observation paths, missed initial localization and other integration fixes; their local raw directories were preserved. None of those development runs is pooled into the frozen five-case result.

## Environment and reproducibility

The [environment manifest](evidence/sil/environment.json) records the host, image ID, installed packages and SHA-256 fingerprints of the actual adapter/core/scripts. Host: Ubuntu 26.04.1, i9-13900K, 32 logical CPUs and 129,209,084 KiB RAM. Runtime: Ubuntu 24.04/Jazzy Docker, Nav2 1.3.12, TurtleBot simulation 1.0.1, ros_gz 1.0.22, Gazebo Harmonic, rosbag2 MCAP 0.26.11, Python 3.12.3 and GCC 13.3.0. Rendering used headless Ogre2/Mesa software; the host GPU was not passed through. The pinned package versions were also confirmed available in the official ROS Noble apt index.

The normal-install qualification began **2026-09-07T03:06:26.296765+00:00** and finished **2026-09-07T03:13:50.169010+00:00**. Cases ran sequentially in separate network/PID/Gazebo partitions, with `OMP_NUM_THREADS=2`. This is a shared local host, without CPU pinning or isolation from unrelated host activity. Three Nav2 stacks execute concurrently but one lease serializes full missions. One fixed seed per case, asynchronous DDS and stochastic MPPI mean these timings cannot establish an optimization, a fault overhead distribution or a population failure rate. The paired nominal case is a control for the workload, not a replicated performance baseline.

Fresh CI had exposed a missing private shared library in normal installation, concealed by the earlier local symlink install. The private core is now statically linked. These results come from a clean **normal colcon install**, after all five physics cases and the contact control were rerun. The earlier symlink-install suite and failed CI record are preserved as development history; they are not pooled into these measurements.

```bash
# From the repository root; requires Linux Docker Engine and Python 3.10+.
# Creates a NEW output directory; builds pinned dependencies and the C++ adapter.
python3 scripts/sil/qualify.py --out runs/sil/fresh-qualification
```

The [protocol](evidence/sil/protocol.json) freezes five cases and thresholds before execution. Each action has a 180-second wall deadline, at most three attempts, and the episode has a 900-second wall deadline. Failure artifacts remain present; a suite receives COMPLETE only after every runner/oracle, the positive control and source-stability check passes. The initial preflight's host symlink-hash error is recorded separately; the final suite ran unchanged source throughout.

## Replay the published evidence

Hash-indexed gzip archives retain original MCAP bags, generated SDF/maps/parameters, topic graphs, action/lease events, telemetry projections and launch logs. [artifacts.json](evidence/sil/artifacts.json) lists their SHA-256 hashes and sizes. Derived models retain [upstream notices](../third_party/NOTICE.md), also included in the archives.

```bash
docker build -t warehouse-fleet:jazzy-sil -f ros2/Dockerfile .
docker run --rm --network none \
  --mount "type=bind,source=$PWD,target=/work" \
  warehouse-fleet:jazzy-sil bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /work/scripts/check_sil_evidence.py'
```

The [archive checker](../scripts/check_sil_evidence.py) checks current source and compressed artifact hashes, extracts to a temporary directory, reads every raw MCAP and compares the independently recomputed reports. It also reads contact/position observations from the positive-control bag. The [replay receipt](evidence/sil/archive-check.json) records the result. A source edit intentionally invalidates the original fingerprint; create new qualification evidence for behavior changes.

Native regression coverage is **37 distinct cases**: 22 C++ core, four CLI integration, one independent discrete-oracle corruption test and ten lease/watchdog/startup-retry cases, passing in both [Release](evidence/sil/release-tests.log) and [ASan/UBSan/LeakSanitizer](evidence/sil/sanitizer-tests.log). A further **eight cases** test the compiled allocation service over actual ROS DDS; [build/test log](evidence/sil/adapter-tests.log). Total: 45 distinct native/ROS service cases. Hosted CI performs native tests and a fresh ROS dependency-image/adapter build with service tests; the five physics episodes are local evidence.

The original [360-run discrete archive was also revalidated](evidence/sil/discrete-archive-check.log). Its 10–50 robot scale and throughput comparisons remain separate from this 1–3 robot physical-simulation qualification.

Readiness can be retried once in a newly created isolated container/world, only before mission start, with no resource owner and only zero recorded final commands. If retried, both attempts are retained, and the startup failure is separately reported and replayed. Runtime mission/fault failures are never retried this way. The final suite used **zero startup retries**; the earlier DDS startup timeout remains in the development record. The guard bounds recovery attempts but does not fix the underlying middleware issue.
