FROM ros:jazzy-ros-base@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive LIBGL_ALWAYS_SOFTWARE=1
# Freeze the ROS dependency cohort; the rolling repository removes older builds.
# Public key fingerprint: 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA.
COPY ros2/ros-snapshots.asc /usr/share/keyrings/ros-snapshots.asc
RUN test "$(gpg --batch --show-keys --with-colons /usr/share/keyrings/ros-snapshots.asc | awk -F: '$1 == "fpr" {print $10; exit}')" = "4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA" \
    && printf 'Types: deb\nURIs: http://snapshots.ros.org/jazzy/2026-06-18/ubuntu\nSuites: noble\nComponents: main\nSigned-By: /usr/share/keyrings/ros-snapshots.asc\n' > /etc/apt/sources.list.d/ros2.sources
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2=1.3.12-1noble.20260615.181551 \
    ros-jazzy-nav2-bringup=1.3.12-1noble.20260616.082701 \
    ros-jazzy-nav2-minimal-tb3-sim=1.0.1-1noble.20260616.074421 \
    ros-jazzy-ros-gz=1.0.22-1noble.20260616.074726 \
    ros-jazzy-rosbag2-storage-mcap=0.26.11-1noble.20260616.074830 python3-pytest \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /work
