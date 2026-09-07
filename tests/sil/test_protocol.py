"""Adversarial safety-state tests; independent of ROS and wall-clock timing."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts/sil"))
from protocol import TransitLease, command_allowed


class LeaseTests(unittest.TestCase):
    def test_cancel_timeout_and_stale_pose_cannot_handoff(self):
        lease = TransitLease()
        generation = lease.acquire(1)
        for succeeded, fresh, x in [
            (False, True, 3),
            (True, False, 3),
            (True, True, 0.9),
            (True, True, math.nan),
        ]:
            self.assertFalse(
                lease.release(
                    1, generation, goal_succeeded=succeeded, fresh_pose=fresh, x=x
                )
            )
            self.assertIsNone(lease.acquire(2))
            self.assertEqual(lease.owner, 1)

    def test_delayed_result_cannot_release_next_generation(self):
        lease = TransitLease()
        old = lease.acquire(1)
        self.assertTrue(
            lease.release(1, old, goal_succeeded=True, fresh_pose=True, x=2)
        )
        current = lease.acquire(1)
        self.assertGreater(current, old)
        self.assertFalse(
            lease.release(1, old, goal_succeeded=True, fresh_pose=True, x=2)
        )
        self.assertIsNone(lease.acquire(2))

    def test_other_robot_cannot_release_ownership(self):
        lease = TransitLease()
        generation = lease.acquire(1)
        self.assertFalse(
            lease.release(2, generation, goal_succeeded=True, fresh_pose=True, x=2)
        )


class GateTests(unittest.TestCase):
    def setUp(self):
        self.valid = {
            "permitted": True,
            "scan_age": 0.1,
            "odom_age": 0.1,
            "command_age": 0.1,
            "permit_age": 0.1,
        }

    def test_healthy_forwarding(self):
        self.assertTrue(command_allowed(**self.valid))

    def test_each_stale_channel_stops(self):
        for field in ["scan_age", "odom_age", "command_age", "permit_age"]:
            with self.subTest(field=field):
                self.assertFalse(command_allowed(**(self.valid | {field: 1.0})))

    def test_nonfinite_and_clock_reversal_stop(self):
        for field in ["scan_age", "odom_age", "command_age", "permit_age"]:
            for age in [math.nan, math.inf, -math.inf, -1.0]:
                self.assertFalse(command_allowed(**(self.valid | {field: age})))

    def test_missing_permission_and_actuator_hold_stop(self):
        self.assertFalse(command_allowed(**(self.valid | {"permitted": False})))
        self.assertFalse(command_allowed(**self.valid, hold=True))


if __name__ == "__main__":
    unittest.main()
