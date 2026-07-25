import os
import tempfile
import unittest

from kinova_interface.motion_lease import MotionLease, MotionLeaseError


class MotionLeaseTests(unittest.TestCase):
    def test_only_one_owner_can_hold_the_process_wide_lease(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "robot-motion.lock")
            first = MotionLease(path, "first")
            second = MotionLease(path, "second")

            self.assertTrue(first.acquire("pick"))
            self.assertFalse(second.acquire("legacy move"))
            first.release()
            self.assertTrue(second.acquire("legacy move"))
            second.release()

    def test_rejects_relative_lock_path(self):
        with self.assertRaises(MotionLeaseError):
            MotionLease("robot-motion.lock", "test")


if __name__ == "__main__":
    unittest.main()
