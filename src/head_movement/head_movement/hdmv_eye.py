from dataclasses import dataclass
from typing import Type, List
from head_movement.hdmv_log import *
from head_movement.hdmv_msg import *

# eye sm states
HDMV_EYE_STATE_NONE   = 0
HDMV_EYE_STATE_MOVING = 1

@dataclass
class hdmv_eye_context:
    current_state = HDMV_EYE_STATE_NONE
    logger: Type[hdmv_logger]
    msg_queue: List[hdmv_msg]

def hdmv_eye_process_msg(ctx, msg) -> bool:
    return False
