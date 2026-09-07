# Generated simulation assets and dependencies

The Gazebo world/robot SDF, ROS robot description and initial Nav2 parameters in generated run artifacts derive from the installed official projects below. Their copyrights remain with their upstream authors; this repository does not claim authorship of those models or frameworks.

- [Nav2 Minimal TurtleBot Simulation](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation), package `nav2_minimal_tb3_sim` 1.0.1, Apache License 2.0. Source templates: `worlds/tb3_sandbox.sdf.xacro`, `urdf/gz_waffle.sdf.xacro`, `urdf/turtlebot3_waffle.urdf`. TurtleBot assets retain the ownership of their original upstream contributors, including ROBOTIS.
- [Navigation2](https://github.com/ros-navigation/navigation2), package `nav2_bringup` 1.3.12, Apache License 2.0. Baseline parameter source: `params/nav2_params.yaml`. Nav2/Gazebo binaries are dependencies and are not authored here.

A copy of [Apache License 2.0](APACHE-2.0.txt) accompanies the derived artifacts. Local modifications generate warehouse walls/maps, add unique robot names and ground-truth/contact sensors, remove unused image sensors, route isolated topics and configure AMCL/controller settings. Original meshes and binary dependency images are not redistributed in the repository.

The independent C++ planning core has no vendored upstream implementation. The project does not adopt a blanket open-source license for its own code by including dependency notices.
