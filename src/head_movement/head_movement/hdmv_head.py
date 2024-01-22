from dataclasses import dataclass
from typing import Type, List
from head_movement.hdmv_log import *
from head_movement.hdmv_msg import *
from head_movement.hdmv_joints import *

import time

from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# head sm states
HDMV_HEAD_STATE_NONE   = 0
HDMV_HEAD_STATE_MOVING = 1

@dataclass
class hdmv_head_context:
    logger: Type[hdmv_logger]
    client: ActionClient
    msg_queue: List[hdmv_msg]

    joint_ids = [\
        joint_id_head_pan_joint,\
        joint_id_head_tilt_right_joint,\
        joint_id_head_tilt_left_joint,\
        joint_id_head_tilt_vertical_joint]

    state = [\
        joint_defval_head_pan_joint, \
        joint_defval_head_tilt_right_joint, \
        joint_defval_head_tilt_left_joint, \
        joint_defval_head_tilt_vertical_joint]

    current_state = HDMV_HEAD_STATE_NONE

def hdmv_head_set_state(ctx, state: int):
    ctx.logger.log_trace(f"head: change state {ctx.current_state} -> {state}")
    ctx.current_state = state

def hdmv_head_push_msg(ctx, msg: hdmv_msg):
    ctx.logger.log_verbose(f"head: push_msg {msg}")
    ctx.msg_queue.append(msg)

def hdmv_head_process_msg(ctx, msg) -> bool:
    result = False;

    if ctx.current_state is HDMV_HEAD_STATE_NONE:
        result = hdmv_head_process_msg_in_none_state(ctx, msg)

    elif ctx.current_state is HDMV_HEAD_STATE_MOVING:
        result = hdmv_head_process_msg_in_moving_state(ctx, msg)

    else:
        ctx.logger.log_warning(f"head in unknown state {ctx.current_state}")
        hdmv_head_set_state(ctx, HDMV_HEAD_STATE_NONE)

    return result

def hdmv_head_handle_msg_head_move_set_target(ctx, msg):
    hdmv_head_set_state(ctx, HDMV_HEAD_STATE_MOVING)
    goal = FollowJointTrajectory.Goal()

    trajectory_points = \
        JointTrajectoryPoint(positions=[-msg.horizontal_tilt, msg.horizontal_tilt], \
                             time_from_start=Duration(sec=1, nanosec=0))

    goal.trajectory = \
        JointTrajectory(joint_names=['head_tilt_left_joint', 'head_tilt_right_joint'],\
                        points=[trajectory_points])

    ctx.logger.log_trace(f"head: move to {msg} in moving state is discarded");
    ctx.client.send_goal_async(goal)

    hdmv_head_set_state(ctx, HDMV_HEAD_STATE_NONE)
    #msg = hdmv_msg(HDMV_MSG_ID_HEAD_MOVE, None)
    #ctx.msg_queue.append(msg)

    time.sleep(1)

    hdmv_head_set_state(ctx, HDMV_HEAD_STATE_NONE)

def hdmv_head_process_msg_in_none_state(ctx, msg) -> bool:
    result = False;

    if msg.msg_id is HDMV_MSG_ID_HEAD_MOVE_SET_TARGET:
        result = True
        hdmv_head_handle_msg_head_move_set_target(ctx, msg)

    else:
        ctx.logger.log_trace(f"head: {msg} in none state is discarded");

    return result

def hdmv_head_process_msg_in_moving_state(ctx, msg) -> bool:
    result = False;

    if msg.msg_id is HDMV_MSG_ID_HEAD_MOVE_SET_TARGET:
        result = True
        hdmv_head_handle_msg_head_move_set_target(ctx, msg)
        #msg = hdmv_msg(HDMV_MSG_ID_HEAD_MOVE, None)
        #hdmv_head_push_msg(ctx, msg)

    if msg.msg_id is HDMV_MSG_ID_HEAD_MOVE:
        result = True
        #msg = hdmv_msg(HDMV_MSG_ID_HEAD_MOVE, None)
        #hdmv_head_push_msg(ctx, msg)

    else:
        ctx.logger.log_trace(f"head: {msg} in moving state is discarded");

    return result
