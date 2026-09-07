# ROS 2 / Gazebo execution layer

This layer runs the original C++ allocator through a typed ROS service, then executes assigned pickup/delivery missions with actual Nav2 actions in Gazebo Harmonic. It is **software-in-the-loop (SIL)**. There is no physical robot, hardware-in-the-loop device, ros2_control implementation, cloud deployment or production safety claim.

## Why this work

The [official JD sample](market-and-technology.md) supports systems fundamentals, reliability, observability and reproducible evaluation for the user's early-career backend/infrastructure profile. Foxglove's robotics infrastructure role and Amazon's experienced fleet role make replayable robotics execution relevant; Amazon remains a stretch-level reference. This addition demonstrates process boundaries, asynchronous failure handling and evidence quality. It does not add Kubernetes, a database or ML workloads already covered by other projects.

| Priority | Engineering value | Acceptance |
|---|---|---|
| Typed C++/ROS boundary | Reuse the existing allocator; reject malformed/bounded requests | Compiled DDS service tests; actual allocations recorded |
| Isolated execution and conservative ownership | Prevent namespace leaks and premature resource handoff | One and three physical simulated robots; feedback-confirmed lease release |
| Fault handling | Stop on stale data; recover without transferring occupied resources | Delayed lidar, actuator hold, killed/respawned Nav2 controller |
| Independent evidence | Detect silent instrumentation errors and false action success | MCAP replay, simulator ground truth, topic graph, deliberate contact positive control |

## Runtime and interfaces

The host is Ubuntu 26.04. The Docker image uses a pinned Ubuntu 24.04 ROS Jazzy base and the installed, qualified Nav2 **1.3.12**, `nav2_minimal_tb3_sim` **1.0.1**, `ros_gz` **1.0.22**, and Gazebo Harmonic. [Dockerfile](../ros2/Dockerfile) pins the top-level Debian packages and base digest; transitive apt dependencies are not a complete historical package snapshot. The evidence manifest records the resulting image digest. This is a compatible runtime addition, not a blanket dependency upgrade. Nav2 1.3.13 is a newer Jazzy release; it is not claimed as tested here.

The [official Gazebo compatibility guide](https://gazebosim.org/docs/harmonic/ros_installation/) pairs Jazzy and Harmonic. The [official Nav2 Gazebo setup](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/gazebo/) and [minimal TurtleBot simulation repository](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation) supply the robot model and baseline configuration. Sources reviewed 2026-09-06 PDT / 2026-09-07 UTC. Version selection stays on this supported pair; the cost is a separate container build and actual integration qualification.

Each `/robotN` namespace contains AMCL, Nav2 servers, robot_state_publisher, and an independent Python command gate. `/clock` is shared; `/robotN/tf` and `/robotN/tf_static` are isolated. Frame names remain `map`, `odom`, `base_footprint`, `base_link` and `base_scan` inside each separate TF bus. Costmap scan topics and collision-monitor footprint topics are explicitly routed; the recorder rejects missing consumers and velocity publishers that bypass the gate.

Gazebo's differential-drive plugin models wheel joints, acceleration and body collisions. The official lidar retains its Gaussian range noise (standard deviation 0.01 m); IMU and wheel odometry are bridged. AMCL receives simulated lidar and odometry, starting from a known approximate spawn pose. The simulator's independent world-frame odometry is **recorded for evaluation only**; it is not fed into AMCL or the coordinator. Camera images are disabled because this navigation stack does not consume them; their collision geometry remains.

The C++ `fleet/allocate` service validates array dimensions, unique identifiers/cells, free endpoints, finite bounded costs and supported strategies. It accepts at most three robots and 32 tasks, returning assignments and two-leg reference routes from the existing allocation/A* code. The physical mission fixture uses one task per robot, with pickup and delivery stations on opposite sides of a warehouse channel. Reference grid routes are inspectable planning outputs; Nav2 replans metric paths to pickup, entry, exit and delivery waypoints. **Discrete time reservations are not projected into physical execution as a safety guarantee.**

## Ownership, stop conditions and recovery

A single in-memory lease conservatively covers the whole mission, including channel approach and exit. All three Nav2 stacks are live, but only the lease owner receives a motion permit. This intentionally serializes missions: it is a small correctness-oriented adapter, not a high-throughput continuous-space multi-agent planner. Disjoint resource concurrency, durable coordinator recovery and fair online admission remain future work.

Every permit has an increasing sequence number and lease generation. Action futures are bounded; rejected/failed goals have at most three attempts. A result belongs to its robot/generation. Cancellation, deadline expiry and old results never release ownership. Release requires a completed delivery action and a fresh AMCL observation outside the channel's protected interval. On unresolved failure the suite denies permits, retains ownership in its result, records the failure and stops its isolated container. Coordinator process restart is not implemented: after a crash the independent gates time out; operators must reconcile positions before creating a new coordinator.

Each gate runs on a steady-clock timer. It forwards `cmd_vel_nav` to the sole final `cmd_vel` publisher only when the permit is current, command age is below 0.35 wall seconds, odometry receipt age below 0.5 wall seconds, and lidar source-stamp age below 0.65 simulation seconds. Permit age is limited to 0.35 wall seconds; nonfinite/implausibly future timestamps fail closed. This is an application watchdog in a simulator, not certified safety control or a real-time OS guarantee.

Fault cases target robot 1 two simulation seconds into its entry action:

* **Lidar delay:** retain original message stamps and delay delivery by one simulation second for four seconds, using a bounded 40-message queue. Cancel the active action, await acknowledgment and terminal status, retain the same lease, wait for fresh sensors, then retry.
* **Actuator hold:** force zero final velocity for four simulation seconds. This emulates a temporary actuator command hold; it is not a calibrated motor fault or a physical obstacle. Verify both zero commands and settled ground-truth motion below 3 cm before recovery.
* **Controller restart:** SIGKILL only robot 1's `controller_server` PID inside the suite's private container. Nav2 launch respawns it and its lifecycle manager restores the stack. The coordinator waits for two active lifecycle snapshots before retry. Logs must contain the old PID's exit and the new PID's startup; final commands must be zero during the outage.

The runner uses a 900-second wall deadline per episode, create-exclusive output directories, isolated network/PID/Gazebo partitions and raw log retention. Runs are sequential. Seeded physics is still nondeterministic because DDS, process scheduling, rendering and MPPI sampling are asynchronous. Qualification times are descriptive, not performance improvements or a fleet reliability rate.

## Evidence and lessons

Actual ROS messages are saved with rosbag2/MCAP: clock, TF, raw/processed scans, IMU, odometry, AMCL, final/pre-gate velocity, contacts, permits and gate events. A separate offline oracle reads the bag instead of trusting the runner's success flag. It checks goal positions against world-frame ground truth, source frames, sample gaps, non-owner commands, task/lease ordering, pair separation and conservative wall clearance. A 0.24 m circumscribed radius covers the model's collision geometry; the 0.48 m pair threshold and 150 ms maximum observed truth gap apply only to this model and sampled local runs.

The positive control deliberately drives a separate simulated robot into a wall. This caught two instrumentation mistakes: the Gazebo contact system used a scoped topic, and the front camera enclosure contacted the wall before the chassis. Instrumentation now covers chassis, camera and lidar links, with geometric wall/pair checks additionally covering wheels. [Gazebo's contact implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc) explains the scoped default and contact-only publication; an empty collision stream alone is not evidence of a functioning sensor. The positive control must pass with the same generated world/bridge before the suite receives a completion marker.

Other development findings: volatile AMCL subscriptions could miss the initial pose before motion, so both coordinator and recorder now request transient-local durability. Westbound goals now carry westbound yaw, avoiding unnecessary end-of-segment reversals. These are integration fixes; development runs were not controlled performance comparisons. The final qualification retains frozen source hashes and all five cases, including any failures.

Local runtime reuse: the machine already had a dependency-only Jazzy/Nav2/Gazebo image from the separate Navigation Reliability Lab project. Its Docker manifest and official-model launch pattern informed this adapter. That project's source and evidence were read only and were not modified. This project's contribution is the original C++ allocation boundary, multi-robot ownership/execution protocol and fleet-specific validation; it does not claim to have authored Nav2, Gazebo or their robot model.

See [SIL qualification](sil-validation.md) for commands, measured results and retained artifacts.

Fresh hosted CI exposed an install-mode defect: ament built the internal planning library as shared, but it was not installed; the local symlink install retained a build-tree search path that concealed the omission. The private core is now explicitly static, and qualification uses normal colcon installation. The entire physics suite was rerun against the installed executable.
