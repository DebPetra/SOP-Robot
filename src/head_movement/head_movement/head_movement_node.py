import math
import sys
import time
import random
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from face_tracker_msgs.msg import Point2, Faces
from std_msgs.msg import Float32

# import sub modules
from head_movement.hdmv_head import *
from head_movement.hdmv_eye import *
from head_movement.hdmv_jaw import *

# import utils
from head_movement.hdmv_log import *
from head_movement.hdmv_cfg import *


class HeadMovementNode(Node):
    def __init__(self):
        super().__init__("head_movement_client")

        self.logger = hdmv_logger(self.get_logger())
        self.msg_queue = []

        self.head_ctx = hdmv_head_context(self.logger, self.msg_queue);
        self.eye_ctx  = hdmv_eye_context(self.logger, self.msg_queue);
        self.jaw_ctx  = hdmv_jaw_context(self.logger, self.msg_queue);

        self.msg_queue.append(hdmv_msg(HDMV_MSG_ID_HEAD_MOVE_SET_TARGET, None))

        # Action clients
        self.eye_action_client = \
            ActionClient(self, \
                         FollowJointTrajectory, \
                         "/eyes_controller/follow_joint_trajectory")

        self.head_action_client = \
            ActionClient(self, \
                         FollowJointTrajectory, \
                         "/head_controller/follow_joint_trajectory")

        # ROS2 subscriptions
        self.sub_face_pos = \
            self.create_subscription(Point2, \
                                     "/face_tracker/face_location_topic", \
                                     self.face_location_update_callback, \
                                     1)

        self.sub_face_list = \
            self.create_subscription(Faces, \
                                     "/face_tracker/face_topic", \
                                     self.face_list_update_callback, \
                                     2)

        self.sub_head_state = \
            self.create_subscription(JointTrajectoryControllerState, \
                                     "/head_controller/state", \
                                     self.head_state_callback, \
                                     5)

        self.sub_eye_state = \
            self.create_subscription(JointTrajectoryControllerState, \
                                     "/eyes_controller/state", \
                                     self.eyes_state_callback, \
                                     5)

        self.sub_head_gesture_len = \
            self.create_subscription(Float32, \
                                     "/head_gestures/length", \
                                     self.head_gesture_callback, \
                                     1)

        # timer for update tick
        self.update_timer = \
            self.create_timer(HDMV_MAIN_LOOP_TICK_RATE, \
                              self.main_loop_callback)

    # end of __init__

    def face_location_update_callback(self, msg):
        """
        callback for /face_tracker/face_location_topic
        msg: Point2
        """
        self.logger.log_trace("on update: /face_tracker/face_location_topic")

    def face_list_update_callback(self, msg):
        """
        callback for /face_tracker/face_topic
        msg: Faces
        """
        self.logger.log_trace("on update: /face_tracker/face_topic");

    def head_state_callback(self, msg):
        """
        callback for /head_controller/state
        msg: JointTrajectoryControllerState
        Get the current state of head joints. Updated at 20 Hz (see robot.yaml)
        """
        self.logger.log_trace("on update: /head_controller/state");

    def eyes_state_callback(self, msg):
        """
        callback for /eyes_controller/state
        msg: JointTrajectoryControllerState
        Get the current state of eye joints. Updated at 20 Hz (see robot.yaml)
        """
        self.logger.log_trace("on update: /eyes_controller/state")

    def head_gesture_callback(self, msg):
        """
        callback for /head_gestures/length
        msg: Float32
        """
        self.logger.log_trace("on update: /head_gestures/length")

    def main_loop_callback(self):
        """
        callback for timer
        executes main loop
        """
        self.logger.log_trace("main loop tick");

        msg_count = len(self.msg_queue);
        i = 0

        while i < msg_count:
            msg = self.msg_queue[i]
            processed = True

            if msg.msg_id >= HDMV_MSG_ID_HEAD_FIRST_ID and \
               msg.msg_id <= HDMV_MSG_ID_HEAD_LAST_ID:

                processed = hdmv_head_process_msg(self.head_ctx, msg);

            elif msg.msg_id >= HDMV_MSG_ID_EYE_FIRST_ID and \
                 msg.msg_id <= HDMV_MSG_ID_EYE_LAST_ID:

                processed = hdmv_eye_process_msg(self.eye_ctx, msg);

            elif msg.msg_id >= HDMV_MSG_ID_JAW_FIRST_ID and \
                 msg.msg_id <= HDMV_MSG_ID_JAW_LAST_ID:

                processed = hdmv_jaw_process_msg(self.jaw_ctx, msg);

            if processed:
                self.msg_queue.pop(i)
                break

# end of class FaceTrackerMovementNode(Node):

def main():
    print("head_movement/head_movement_nod:main()");

    rclpy.init()

    action_client = HeadMovementNode()
    rclpy.spin(action_client)

    # Shutdown
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
