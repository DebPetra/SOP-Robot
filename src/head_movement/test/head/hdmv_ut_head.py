import unittest
from head_movement.head_movement_node import *
import rclpy

import threading
import time
import subprocess

from face_tracker_msgs.msg import Point2, Faces

"""
How are tests implemented:

One possibility is to publish messages directly to the ROS node, and
spin it with rclpy.init(). This is hovever tricky since it is
impossible to check the current state accurately.

Other option, and which is deployed here is to call callback functions
direcly. This way we can check the state after every message handling
iteration.
"""
class UT_HeadMovementNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        command = ["ros2", "launch", "robot", "robot.fake.launch.py"]
        cls.process = subprocess.Popen(command)
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        cls.process.kill()
        rclpy.shutdown()

    def setUp(self):
        self.node = HeadMovementNode(enable_logging=True)
        self.node.logger.ENABLE_TRACE   = True
        self.node.logger.ENABLE_VERBOSE = True
        self.node.logger.ENABLE_WARNING = True
        self.node.logger.ENABLE_FATAL   = True

    def tearDown(self):
        self.node.close()

    def test_initialization(self):
        self.assertEqual(self.node.head_ctx.current_state, HDMV_HEAD_STATE_NONE)
        self.assertEqual(self.node.eye_ctx.current_state, HDMV_EYE_STATE_NONE)
        self.assertEqual(self.node.jaw_ctx.current_state, HDMV_JAW_STATE_NONE)

        self.assertEqual(len(self.node.msg_queue), 0)

    def test_initialization2(self):
        self.assertEqual(self.node.head_ctx.current_state, HDMV_HEAD_STATE_NONE)
        self.assertEqual(self.node.eye_ctx.current_state, HDMV_EYE_STATE_NONE)
        self.assertEqual(self.node.jaw_ctx.current_state, HDMV_JAW_STATE_NONE)

        self.assertEqual(len(self.node.msg_queue), 0)


if __name__ == "__main__":
    print("Running tests")
    unittest.main()
