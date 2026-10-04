# Rebuilding the qualified ROS dependency cohort

The top-level Debian versions in [ros2/Dockerfile](../ros2/Dockerfile) belong to the ROS Jazzy 2026-06-18 sync. The rolling `packages.ros.org` repository later replaced these builds, causing fresh hosted CI images to fail before any project tests ran.

The image now uses the official [Jazzy 2026-06-18 snapshot](http://snapshots.ros.org/jazzy/2026-06-18/ubuntu/), keeping the base digest and all five exact package versions unchanged. ROS transitive packages are resolved from that same snapshot. Ubuntu packages still receive their configured Noble updates; this is not a claim that every Debian package is frozen or that the new image is byte-identical to a historical image.

APT continues to require valid repository signatures and package hashes. The snapshot uses a separate ROS public signing key, checked into [ros2/ros-snapshots.asc](../ros2/ros-snapshots.asc), with fingerprint `4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA`; the Docker build verifies that full fingerprint before installing anything. This is a public verification key, not a credential. Its [source](https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA) and fingerprint are documented by [Open Robotics](https://discourse.ros.org/t/ros-signing-key-migration-guide/43937/41). The snapshot currently serves HTTP; authenticity comes from APT's mandatory signature verification. No `trusted=yes`, unauthenticated installation, or expired-signature bypass is enabled.

When that signing key is renewed, retrieve the public key using the full fingerprint, inspect the fingerprint and expiry, and review the change before replacing the checked-in key. A missing snapshot or failed signature should stop the build rather than silently select a newer unqualified ROS cohort.

## Reproduce

```sh
docker build -t warehouse-fleet-lab:snapshot -f ros2/Dockerfile .
```

Run the existing workflow's DDS and simulation commands against the rebuilt image. This dependency repair does not change the Experience Bank results or qualify a new Nav2 release.

## Local validation — 2026-10-04 UTC

The dependency image rebuilt successfully from the pinned base using signed snapshot metadata. On a clean exported source tree, the existing CI ROS command built both overlay packages and passed eight allocator DDS cases, 17 lifecycle DDS cases, the unmodified Nav2 missing-response hang control, and three readiness cases. The original hang was reproduced as the expected negative control. Native planner jobs had already passed in the original hosted run; this local dependency check did not rerun the six physics episodes or the 360-run discrete matrix.
