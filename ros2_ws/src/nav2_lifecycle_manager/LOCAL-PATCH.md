# Local lifecycle response recovery patch

Base: official Navigation2 tag `1.3.12`, package `nav2_lifecycle_manager`.
`UPSTREAM.json` retains SHA-256 digests of all original downloaded files.
The original copyright headers and Apache-2.0 license remain in place.

Changes to upstream files:

* `include/nav2_lifecycle_manager/lifecycle_manager.hpp`: use the local reconciled client and retain its budgets.
* `src/lifecycle_manager.cpp`: declare and validate finite RPC/transition budgets; construct clients without discovery blocking; use their state-confirmed outcome. The upstream bond monitoring, ordering and respawn logic remain in use.
* `CMakeLists.txt`: compile the new client and install the local DDS regression executable and test-only reply-suppression proxy. The upstream test directory is not vendored or claimed tested.

Added files implement bounded RPC/discovery, read-only reconciliation and deliberately missing/late responses. The client sends at most one mutation per transition call, honors negative responses and removes timed-out pending requests. A state-changing request can still execute after its caller times out; the patch does not cancel it or blindly resend it. Unknown outcomes leave the manager inactive. One serialized lifecycle owner per node is required.

This is a normal colcon overlay of the installed 1.3.12 package, qualified only in this project's pinned container with `use_composition=False`. The public manager-client interface is unchanged; the manager class internals/ABI change. External components that directly instantiate the manager must be rebuilt. Do not copy this shared library into an unrelated ROS installation. The installed dependency versions and patched binary/library hashes are recorded with each SIL suite.

The ROS middleware itself is unchanged. This fixes the application's unbounded wait and recovers confirmed completed transitions; it does not establish a fix for Fast DDS discovery races.
