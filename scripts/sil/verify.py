#!/usr/bin/env python3
"""Independent MCAP oracle: measured motion, commands, sensor age and mission/lease events.

Run inside the pinned ROS image. No planner or coordinator implementation is imported.
This is sampled simulation evidence, not a proof of real-world collision avoidance.
"""

import argparse
import itertools
import json
import math
import re
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def verify(out):
    out = Path(out)
    run = json.loads((out / "result.json").read_text())
    events = [
        json.loads(line) for line in (out / "events.jsonl").read_text().splitlines()
    ]
    graph = json.loads((out / "graph.json").read_text())
    errors = []

    def require(condition, message):
        if not condition:
            errors.append(message)

    require(run["passed"], "runner did not complete")
    require(
        (out / "launch.log")
        .read_text()
        .count("FLEET_LIFECYCLE_PATCH bounded_state_reconciliation_v1")
        >= 2 * run["robots"],
        "patched localization/navigation lifecycle managers were not observed",
    )
    robots = range(1, run["robots"] + 1)
    for robot in robots:
        ns = f"/robot{robot}"
        require(
            graph[ns + "/cmd_vel"]["publishers"] == [ns + "/command_gate"],
            f"{ns}: final command bypass",
        )
        require(
            graph[ns + "/scan_raw"]["subscribers"]
            == ["/fleet_coordinator", ns + "/command_gate"],
            f"{ns}: raw scan bypass",
        )
        for subscriber in [
            "amcl",
            "collision_monitor",
            "local_costmap/local_costmap",
            "global_costmap/global_costmap",
        ]:
            require(
                ns + "/" + subscriber in graph[ns + "/scan"]["subscribers"],
                f"{ns}: missing processed scan subscriber {subscriber}",
            )
        require(
            all(
                name == "/fleet_coordinator" or name.startswith(ns + "/")
                for name in graph[ns + "/tf"]["subscribers"]
            ),
            f"{ns}: TF namespace leak",
        )
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(out / "bag"), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {
        topic.name: get_message(topic.type)
        for topic in reader.get_all_topics_and_types()
    }
    counts = Counter()
    truth = defaultdict(list)
    commands = defaultdict(list)
    gate_events = defaultdict(list)
    scan_ages = defaultdict(list)
    contact_events = []
    frame_errors = set()
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        counts[topic] += 1
        if not topic.startswith("/robot"):
            continue
        robot = int(topic.split("/")[1][5:])
        msg = deserialize_message(data, types[topic])
        time = stamp / 1e9
        if topic.endswith("/ground_truth/odom"):
            p = msg.pose.pose.position
            truth[robot].append((time, p.x, p.y))
            if msg.header.frame_id != "world":
                frame_errors.add((topic, msg.header.frame_id))
        elif topic.endswith("/odom") and msg.header.frame_id != "odom":
            frame_errors.add((topic, msg.header.frame_id))
        elif topic.endswith("/cmd_vel"):
            commands[robot].append((time, msg.linear.x, msg.angular.z))
        elif topic.endswith("/scan"):
            scan_ages[robot].append(
                (time, time - msg.header.stamp.sec - msg.header.stamp.nanosec / 1e9)
            )
            if msg.header.frame_id != "base_scan":
                frame_errors.add((topic, msg.header.frame_id))
        elif topic.endswith("/gate/events"):
            gate_events[robot].append(json.loads(msg.data))
        elif topic.rsplit("/", 1)[-1].startswith("contacts"):
            for contact in msg.contacts:
                contact_events.append(
                    {
                        "time": time,
                        "robot": robot,
                        "a": contact.collision1.name,
                        "b": contact.collision2.name,
                    }
                )
    require(not frame_errors, f"unexpected frames: {sorted(frame_errors)}")
    require(
        dict(counts) == run["topic_counts"], "MCAP counts differ from recorded receipt"
    )
    mission_start, mission_end = (
        run.get("mission_start_sim", 0),
        run.get("mission_end_sim", math.inf),
    )
    paths = {}
    max_gaps = {}
    for robot in robots:
        for suffix, minimum in [
            ("ground_truth/odom", 200),
            ("odom", 100),
            ("scan", 20),
            ("scan_raw", 20),
            ("tf", 100),
            ("tf_static", 1),
            ("amcl_pose", 3),
            ("cmd_vel_nav", 20),
        ]:
            require(
                counts[f"/robot{robot}/{suffix}"] >= minimum,
                f"robot{robot}: insufficient {suffix}",
            )
        samples = truth[robot]
        require(
            all(math.isfinite(v) for sample in samples for v in sample),
            "nonfinite ground truth",
        )
        paths[robot] = sum(
            math.hypot(b[1] - a[1], b[2] - a[2])
            for a, b in itertools.pairwise(samples)
            if a[0] >= mission_start and b[0] <= mission_end
        )
        max_gaps[robot] = max(
            (
                b[0] - a[0]
                for a, b in itertools.pairwise(samples)
                if a[0] >= mission_start
            ),
            default=math.inf,
        )
        require(
            max_gaps[robot] < 0.15, f"robot{robot}: ground truth sampling gap >=150ms"
        )
    # Pair distances from nearest preceding observations with <=100ms skew.
    last = {}
    min_distance = math.inf
    pair_samples = 0
    for time, robot, x, y in sorted(
        (t, r, x, y) for r, rows in truth.items() for t, x, y in rows
    ):
        for other, (ot, ox, oy) in last.items():
            if other != robot and 0 <= time - ot < 0.1:
                min_distance = min(min_distance, math.hypot(x - ox, y - oy))
                pair_samples += 1
        last[robot] = (time, x, y)
    if run["robots"] > 1:
        require(
            pair_samples > 1000 and min_distance >= 0.48,
            "sampled robot pair separation below 0.48m or insufficient pairs",
        )
    require(not contact_events, f"Gazebo chassis contact events: {len(contact_events)}")
    # Circumscribed 0.24m footprint includes wheels/camera, independently of contacts.
    walls = []
    for model in ET.parse(out / "generated/world.sdf").getroot().findall("world/model"):
        if model.get("name", "").startswith("wall_"):
            x, y, *_ = map(float, model.findtext("pose").split())
            sx, sy, _ = map(
                float, model.findtext("link/collision/geometry/box/size").split()
            )
            walls.append((x, y, sx, sy))
    require(len(walls) == 6, "expected six warehouse walls")
    min_wall_distance = min(
        (
            math.hypot(max(abs(x - bx) - sx / 2, 0), max(abs(y - by) - sy / 2, 0))
            for rows in truth.values()
            for _, x, y in rows
            for bx, by, sx, sy in walls
        ),
        default=0,
    )
    require(min_wall_distance >= 0.24, "sampled footprint intersects warehouse wall")
    owner = None
    generation = 0
    completion_errors = []
    completed = set()
    for event in events:
        kind = event["kind"]
        if kind == "lease_acquired":
            require(
                owner is None and event["generation"] > generation,
                "overlapping/stale lease acquisition",
            )
            owner, generation = event["robot"], event["generation"]
        elif kind in ("goal_sent", "goal_result", "cancel_acknowledged", "retry"):
            require(
                owner == event["robot"] and generation == event["generation"],
                "action without current resource ownership",
            )
        elif kind == "lease_released":
            require(
                owner == event["robot"]
                and generation == event["generation"]
                and abs(event["amcl_x"]) >= 1.2
                and event["pose_age"] < 2,
                "unconfirmed lease release",
            )
            owner = None
        elif kind == "task_completed":
            require(event["task"] not in completed, "duplicate completion")
            completed.add(event["task"])
            nearest = min(
                truth[event["robot"]], key=lambda p: abs(p[0] - event["sim_time"])
            )
            error = math.hypot(
                nearest[1] - event["target"][0], nearest[2] - event["target"][1]
            )
            completion_errors.append(error)
            require(
                abs(nearest[0] - event["sim_time"]) < 0.1 and error <= 0.45,
                "goal success disagrees with Gazebo ground truth",
            )
    require(
        owner is None and completed == set(robots),
        "incomplete tasks or retained resource",
    )
    # During each lease, commands for every other robot must stay zero.
    intervals = []
    for acquired in [e for e in events if e["kind"] == "lease_acquired"]:
        released = next(
            (
                e
                for e in events
                if e["kind"] == "lease_released"
                and e["generation"] == acquired["generation"]
            ),
            None,
        )
        if released:
            intervals.append(
                (acquired["sim_time"], released["sim_time"], acquired["robot"])
            )
    unauthorized = sum(
        1
        for robot, rows in commands.items()
        for t, v, w in rows
        if abs(v) + abs(w) > 1e-5
        and any(
            start + 0.3 < t < end and robot != owner for start, end, owner in intervals
        )
    )
    require(unauthorized == 0, "non-owner received nonzero commands")
    fault = run["fault"]
    fault_metrics = {}
    if fault == "lifecycle_reply_loss":
        log = (out / "launch.log").read_text()
        dropped = (
            "FLEET_REPLY_PROXY configured_real_map_server=true response_suppressed=true"
        )
        reconciled = "node=map_server outcome=reconciled target=2"
        require(
            log.count(dropped) == 1,
            "real map-server reply was not suppressed exactly once",
        )
        require(reconciled in log, "missing lifecycle state reconciliation")
        require(
            log.count("FLEET_REPLY_PROXY forwarded_transition=1") == 1,
            "configure was not forwarded exactly once",
        )
        require(
            log.count("FLEET_REPLY_PROXY forwarded_transition=3") == 1,
            "activate was not forwarded exactly once",
        )
        initial_commands = [
            (v, w)
            for rows in commands.values()
            for t, v, w in rows
            if t < mission_start
        ]
        require(
            len(initial_commands) >= 20
            and all(abs(v) + abs(w) < 1e-5 for v, w in initial_commands),
            "motion before lifecycle startup completed",
        )
        fault_metrics = {
            "suppressed_real_configure_responses": log.count(dropped),
            "configure_requests_forwarded": log.count(
                "FLEET_REPLY_PROXY forwarded_transition=1"
            ),
            "pre_mission_zero_commands": len(initial_commands),
            "state_reconciled_without_world_restart": reconciled in log,
        }
    elif fault != "none":
        requests = [e for e in events if e["kind"] == "fault_requested"]
        require(
            len(requests) == 1 and requests[0]["fault"] == fault,
            "fault request missing/duplicated",
        )
        if requests:
            begin = requests[0]["sim_time"]
            if fault in ("scan_delay", "actuator_hold"):
                starts = [
                    e
                    for e in gate_events[1]
                    if e["kind"] == "fault_started" and e["fault"] == fault
                ]
                ends = [
                    e
                    for e in gate_events[1]
                    if e["kind"] == "fault_cleared" and e["fault"] == fault
                ]
                require(
                    len(starts) == 1 and len(ends) == 1,
                    "gate did not observe complete fault interval",
                )
                if starts and ends:
                    begin, end = starts[0]["sim_time"], ends[0]["sim_time"]
                    window = [(v, w) for t, v, w in commands[1] if begin + 1 < t < end]
                    require(
                        len(window) >= 10
                        and all(abs(v) + abs(w) < 1e-5 for v, w in window),
                        "fault did not hold final command at zero",
                    )
                    require(
                        any(t > end and abs(v) > 0.05 for t, v, w in commands[1]),
                        "no motion after fault cleared",
                    )
                    fault_metrics.update(
                        observed_duration_seconds=end - begin,
                        zero_command_samples=len(window),
                    )
                    held = [(x, y) for t, x, y in truth[1] if begin + 1 < t < end]
                    drift = max(
                        (math.hypot(x - held[0][0], y - held[0][1]) for x, y in held),
                        default=math.inf,
                    )
                    require(
                        drift < 0.03,
                        "robot moved more than 3cm during settled fault hold",
                    )
                    fault_metrics["settled_hold_drift_meters"] = drift
                    if fault == "scan_delay":
                        age = max(
                            (age for t, age in scan_ages[1] if begin < t < end),
                            default=0,
                        )
                        require(
                            age >= 0.95, "scan delay not observed in message timestamps"
                        )
                        require(
                            any(e["kind"] == "cancel_acknowledged" for e in events)
                            and any(e["kind"] == "retry" for e in events),
                            "cancel/retry not exercised",
                        )
                        fault_metrics["max_observed_scan_age_seconds"] = age
            else:
                log = (out / "launch.log").read_text()
                pid = requests[0]["pid"]
                require(
                    f"pid {pid}, exit code -9" in log,
                    "killed controller exit missing from launch log",
                )
                pids = re.findall(
                    r"\[controller_server-\d+\]: process started with pid \[(\d+)\]",
                    log,
                )
                require(
                    len(set(pids)) > run["robots"], "controller respawn not observed"
                )
                require(
                    any(t > begin + 3 and abs(v) > 0.05 for t, v, w in commands[1]),
                    "no motion after controller restart",
                )
                fault_metrics["controller_process_ids"] = pids
                zeros = [
                    (v, w) for t, v, w in commands[1] if begin + 0.5 < t < begin + 2
                ]
                require(
                    len(zeros) >= 10 and all(abs(v) + abs(w) < 1e-5 for v, w in zeros),
                    "nonzero commands during controller outage",
                )
                fault_metrics["outage_zero_command_samples"] = len(zeros)
    return {
        "passed": not errors,
        "errors": errors,
        "robots": run["robots"],
        "fault": fault,
        "seed": run["seed"],
        "completed_tasks": len(completed),
        "mission_sim_seconds": mission_end - mission_start,
        "wall_seconds": run["wall_seconds"],
        "ground_truth_samples": sum(len(v) for v in truth.values()),
        "max_ground_truth_gap_seconds": max_gaps,
        "path_length_meters": paths,
        "pair_samples": pair_samples,
        "min_pair_distance_meters": min_distance
        if math.isfinite(min_distance)
        else None,
        "chassis_contact_events": len(contact_events),
        "min_center_to_wall_meters": min_wall_distance,
        "max_delivery_error_meters": max(completion_errors, default=None),
        "unauthorized_nonzero_commands": unauthorized,
        "fault_observations": fault_metrics,
        "bag_message_count": sum(counts.values()),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    args = parser.parse_args()
    result = verify(args.episode)
    (args.episode / "verification.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    print(json.dumps(result, indent=2, allow_nan=False))
    raise SystemExit(0 if result["passed"] else 1)
