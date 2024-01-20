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
from head_movement.hdmv_joints import *

class HeadMovementNode(Node):
    def __init__(self):
        super().__init__("head_movement_client")

        # Action clients
        eye_action_client = \
            ActionClient(self, \
                         FollowJointTrajectory, \
                         "/eyes_controller/follow_joint_trajectory")

        head_action_client = \
            ActionClient(self, \
                         FollowJointTrajectory, \
                         "/head_controller/follow_joint_trajectory")

        self.logger = hdmv_logger(self.get_logger())
        self.msg_queue = []

        self.head_ctx = hdmv_head_context(self.logger, head_action_client, self.msg_queue);
        self.eye_ctx  = hdmv_eye_context(self.logger, eye_action_client, self.msg_queue);
        self.jaw_ctx  = hdmv_jaw_context(self.logger, self.msg_queue);

        self.msg_queue.append(hdmv_msg(HDMV_MSG_ID_HEAD_MOVE_SET_TARGET, None))

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

        self.set_eye_move_target(mgs.x, msg.y);
        self.set_head_move_target(msg.x, msg.y);

    def set_eye_move_target(self, x, y):
        x_diff = HDMV_FACE_TRACKER_HALF_WIDTH - face_location_x
        y_diff = HDMV_FACE_TRACKER_HALF_HEIGHT - face_location_y

        eye_location_x = x_diff * HDMV_EYE_MOVE_HORIZONTAL_COEFF + self.eye_ctx.state[0];
        eye_location_y = y_diff * HDMV_EYE_MOVE_VERTICAL_COEFF + self.eye_ctx.state[0];

        # hmm, what is this for
        eye_location_y = max(min(-0.2, eye_location_y), -0.7)
        msg = hdmv_msg_eye_move_set_target(eye_location_x, eye_location_y)
        self.msg_queue.append(msg)

    def set_head_move_target(self, x, y):
                # Calculate face movement
        x_diff = self.middle_x - face_location_x
        y_diff = self.middle_y - face_location_y

        # If the face is close enough to the center, leave the small movements for the eyes.
        if abs(x_diff) < 100:
            x_diff = 0
        if abs(y_diff) < 50:
            y_diff = 0

        h_coeff = HDMV_HEAD_MOVE_HORINTAL_COEFF

        pan = x_diff * h_coeff + self.head_ctx.state[0]
        pan = max(min(1.75, pan), -0.25)

        # Vertical tilt
        v_coeff = HDMV_HEAD_MOVE_VERTICAL_COEFF
        vertical_tilt = y_diff * v_coeff + self.head_ctx.state[3]
        vertical_tilt = max(min(1.5, vertical_tilt), 0.8)

        msg = hdmv_msg_head_move_set_target(pan, vertical_tilt, self.head_ctx.state[1])
        self.msg_queue.append(msg)

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
        for i, val in enumerate(msg.actual.positions):
            if math.isnan(val):
                self.head_ctx.state[i] = joint_id_to_defval(self.head_ctx.joint_ids[i])
                self.logger.log_warning(f"head joint_id {self.head_ctx.joint_ids[i]} is not responding")
            else:
                self.head_state[i] = val


    def eyes_state_callback(self, msg):
        """
        callback for /eyes_controller/state
        msg: JointTrajectoryControllerState
        Get the current state of eye joints. Updated at 20 Hz (see robot.yaml)
        """
        self.logger.log_trace("on update: /eyes_controller/state")
        for i, val in enumerate(msg.actual.positions):
            if math.isnan(val):
                self.eye_ctx.state[i] = joint_id_to_defval(self.eye_ctx.joint_ids[i])
                self.logger.log_warning(f"eye joint_id {self.eye_ctx.joint_ids[i]} is not responding")
            else:
                self.head_state[i] = val

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

            self.logger.log_verbose(f"processing [{i}] msg {msg}")

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





