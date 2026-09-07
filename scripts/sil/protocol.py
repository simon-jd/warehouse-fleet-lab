"""Pure state contracts shared by the ROS coordinator and deterministic fault tests."""

import math
from dataclasses import dataclass


@dataclass
class TransitLease:
    owner: int | None = None
    generation: int = 0

    def acquire(self, robot):
        if self.owner is not None:
            return None
        self.owner = robot
        self.generation += 1
        return self.generation

    def release(self, robot, generation, *, goal_succeeded, fresh_pose, x):
        if robot != self.owner or generation != self.generation:
            return False
        # A timeout/cancel/old result never authorizes handing the channel to another robot.
        if not goal_succeeded or not fresh_pose or not math.isfinite(x) or abs(x) < 1.2:
            return False
        self.owner = None
        return True


def command_allowed(
    *, permitted, scan_age, odom_age, command_age, permit_age, hold=False
):
    ages = (scan_age, odom_age, command_age, permit_age)
    return (
        permitted
        and not hold
        and all(math.isfinite(x) and x >= -0.05 for x in ages)
        and scan_age < 0.65
        and odom_age < 0.5
        and command_age < 0.35
        and permit_age < 0.35
    )
