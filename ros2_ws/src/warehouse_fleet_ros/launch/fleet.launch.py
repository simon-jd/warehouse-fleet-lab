"""Namespace-isolated Jazzy/Nav2 stacks and Gazebo Harmonic physics."""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def setup(context):
    package = Path(get_package_share_directory("warehouse_fleet_ros"))
    sys.path.insert(0, str(package.parent.parent / "lib/warehouse_fleet_ros"))
    from world import STARTS, WALLS, box, write_maps

    sim = Path(get_package_share_directory("nav2_minimal_tb3_sim"))
    nav = Path(get_package_share_directory("nav2_bringup"))
    arg = lambda name: LaunchConfiguration(name).perform(context)
    count = int(arg("robots"))
    if count not in (1, 3):
        raise ValueError("robots must be 1 or 3")
    out = Path(arg("run_dir")).resolve()
    out.mkdir(parents=True, exist_ok=True)
    write_maps(out)
    world = ET.fromstring(
        xacro.process_file(
            str(sim / "worlds/tb3_sandbox.sdf.xacro"), mappings={"headless": "true"}
        ).toxml()
    )
    world_node = world.find("world")
    world_node.remove(world_node.find("model[@name='turtlebot3_world']"))
    for i, dimensions in enumerate(WALLS):
        world_node.append(box(f"wall_{i}", *dimensions))
    world_node.append(
        ET.fromstring(
            '<plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>'
        )
    )
    bridge = [
        {
            "topic_name": "/clock",
            "ros_type_name": "rosgraph_msgs/msg/Clock",
            "gz_type_name": "gz.msgs.Clock",
            "direction": "GZ_TO_ROS",
        }
    ]
    actions = []
    for i, (x, y) in enumerate(STARTS[:count], 1):
        ns = f"robot{i}"
        robot = ET.fromstring(
            xacro.process_file(
                str(sim / "urdf/gz_waffle.sdf.xacro"), mappings={"namespace": "/" + ns}
            ).toxml()
        ).find("model")
        robot.set("name", ns)
        robot.find("pose").text = f"{x} {y} .01 0 0 0"
        # Camera rendering is unnecessary: navigation consumes lidar and odometry.
        for link in robot.findall("link"):
            for sensor in list(link.findall("sensor")):
                if sensor.find("camera") is not None:
                    link.remove(sensor)
        robot.append(
            ET.fromstring(f"""<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
          <odom_frame>world</odom_frame><robot_base_frame>base_footprint</robot_base_frame>
          <odom_topic>/{ns}/ground_truth/odom</odom_topic><odom_publish_frequency>50</odom_publish_frequency><dimensions>2</dimensions></plugin>""")
        )
        robot.find("link[@name='base_link']").append(
            ET.fromstring(f"""<sensor name="chassis_contact" type="contact">
          <always_on>true</always_on><update_rate>50</update_rate><topic>/{ns}/contacts</topic>
          <contact><collision>base_collision</collision></contact></sensor>""")
        )
        world_node.append(robot)
        for body, suffix in [("camera_link", "camera"), ("base_scan", "lidar")]:
            link = robot.find(f"link[@name='{body}']")
            collision = link.find("collision").get("name")
            link.append(
                ET.fromstring(f"""<sensor name="body_contact" type="contact">
                <always_on>true</always_on><update_rate>50</update_rate>
                <contact><collision>{collision}</collision></contact></sensor>""")
            )
            bridge.append(
                {
                    "ros_topic_name": f"/{ns}/contacts_{suffix}",
                    "gz_topic_name": f"/world/default/model/{ns}/link/{body}/sensor/body_contact/contact",
                    "ros_type_name": "ros_gz_interfaces/msg/Contacts",
                    "gz_type_name": "gz.msgs.Contacts",
                    "direction": "GZ_TO_ROS",
                }
            )
        topics = [
            ("joint_states", "sensor_msgs/msg/JointState", "Model"),
            ("odom", "nav_msgs/msg/Odometry", "Odometry"),
            ("tf", "tf2_msgs/msg/TFMessage", "Pose_V"),
            ("imu", "sensor_msgs/msg/Imu", "IMU"),
            ("scan", "sensor_msgs/msg/LaserScan", "LaserScan"),
            ("cmd_vel", "geometry_msgs/msg/Twist", "Twist"),
            ("ground_truth/odom", "nav_msgs/msg/Odometry", "Odometry"),
            ("contacts", "ros_gz_interfaces/msg/Contacts", "Contacts"),
        ]
        for topic, ros_type, gz_type in topics:
            bridge.append(
                {
                    "ros_topic_name": f"/{ns}/"
                    + ("scan_raw" if topic == "scan" else topic),
                    "gz_topic_name": (
                        f"/world/default/model/{ns}/link/base_link/sensor/chassis_contact/contact"
                        if topic == "contacts"
                        else f"/{ns}/{topic}"
                    ),
                    "ros_type_name": ros_type,
                    "gz_type_name": "gz.msgs." + gz_type,
                    "direction": "ROS_TO_GZ" if topic == "cmd_vel" else "GZ_TO_ROS",
                }
            )
        params = yaml.safe_load((nav / "params/nav2_params.yaml").read_text())
        params["amcl"]["ros__parameters"].update(
            {
                "set_initial_pose": True,
                "initial_pose": {"x": x, "y": y, "z": 0.0, "yaw": 0.0},
                "min_particles": 300,
                "max_particles": 1000,
                "update_min_d": 0.1,
                "update_min_a": 0.1,
            }
        )
        params["bt_navigator"]["ros__parameters"]["odom_topic"] = "odom"
        controller = params["controller_server"]["ros__parameters"]
        controller["FollowPath"].update(
            {"batch_size": 500, "visualize": False, "vx_max": 0.4}
        )
        controller["progress_checker"]["movement_time_allowance"] = 15.0
        params["collision_monitor"]["ros__parameters"]["cmd_vel_out_topic"] = (
            "cmd_vel_nav"
        )
        params["velocity_smoother"]["ros__parameters"]["odom_topic"] = "odom"
        params["collision_monitor"]["ros__parameters"]["FootprintApproach"][
            "footprint_topic"
        ] = "local_costmap/published_footprint"
        for costmap, layer in [
            ("local_costmap", "voxel_layer"),
            ("global_costmap", "obstacle_layer"),
        ]:
            params[costmap][costmap]["ros__parameters"][layer]["scan"]["topic"] = (
                f"/{ns}/scan"
            )
        config = out / f"{ns}.yaml"
        config.write_text(yaml.safe_dump(params))
        actions.extend(
            [
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    namespace=ns,
                    parameters=[
                        {
                            "use_sim_time": True,
                            "robot_description": (
                                sim / "urdf/turtlebot3_waffle.urdf"
                            ).read_text(),
                        }
                    ],
                    remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                ),
                Node(
                    package="warehouse_fleet_ros",
                    executable="command_gate.py",
                    namespace=ns,
                    parameters=[{"use_sim_time": True}],
                ),
                GroupAction(
                    [
                        SetRemap(src="docking_server:cmd_vel", dst="cmd_vel_nav"),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                str(nav / "launch/bringup_launch.py")
                            ),
                            launch_arguments={
                                "namespace": ns,
                                "use_namespace": "True",
                                "use_sim_time": "True",
                                "autostart": "True",
                                "use_composition": "False",
                                "use_respawn": "True",
                                "map": str(out / "map.yaml"),
                                "params_file": str(config),
                                "log_level": "warn",
                            }.items(),
                        ),
                    ]
                ),
            ]
        )
    ET.ElementTree(world).write(out / "world.sdf", encoding="unicode")
    (out / "bridge.yaml").write_text(yaml.safe_dump(bridge))
    return [
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", str(sim / "models") + ":" + str(sim.parent)
        ),
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
        ExecuteProcess(
            cmd=[
                "gz",
                "sim",
                "-r",
                "-s",
                "--headless-rendering",
                "--seed",
                arg("seed"),
                str(out / "world.sdf"),
            ],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="fleet_bridge",
            parameters=[
                {"config_file": str(out / "bridge.yaml"), "use_sim_time": True}
            ],
        ),
        Node(
            package="warehouse_fleet_ros",
            executable="allocator_node",
            parameters=[{"grid_map": str(out / "grid.map")}],
        ),
        *actions,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(name, default_value=value)
            for name, value in {
                "run_dir": "/tmp/fleet-sil",
                "robots": "1",
                "seed": "41",
            }.items()
        ]
        + [OpaqueFunction(function=setup)]
    )
