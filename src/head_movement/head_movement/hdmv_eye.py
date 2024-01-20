from dataclasses import dataclass
from typing import Type, List
from head_movement.hdmv_log import *
from head_movement.hdmv_msg import *

from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# eye sm states
HDMV_EYE_STATE_NONE   = 0
HDMV_EYE_STATE_MOVING = 1

@dataclass
class hdmv_eye_context:
    logger: Type[hdmv_logger]
    client: ActionClient
    msg_queue: List[hdmv_msg]

    current_state = HDMV_EYE_STATE_NONE

def hdmv_eye_process_msg(ctx, msg) -> bool:
    return False
