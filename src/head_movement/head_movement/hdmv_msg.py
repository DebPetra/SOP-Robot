from dataclasses import dataclass

@dataclass
class hdmv_msg:
    msg_id: int
    data: any

########################################
# head messages
########################################
HDMV_MSG_ID_HEAD_FIRST_ID        = 100
HDMV_MSG_ID_HEAD_MOVE_SET_TARGET = 100
HDMV_MSG_ID_HEAD_MOVE            = 101
HDMV_MSG_ID_HEAD_LAST_ID         = 199

@dataclass
class hdmv_msg_head_move_set_target:
    pan: int
    v_tilt: int
    h_tilt: int

########################################
# eye messages
########################################
HDMV_MSG_ID_EYE_FIRST_ID        = 200
HDMV_MSG_ID_EYE_MOVE_SET_TARGET = 200
HDMV_MSG_ID_EYE_MOVE            = 201
HDMV_MSG_ID_EYE_LAST_ID         = 299

@dataclass
class hdmv_msg_eye_move_set_target:
    x: int
    y: int

########################################
# jaw messages
########################################
HDMV_MSG_ID_JAW_FIRST_ID        = 300
HDMV_MSG_ID_JAW_MOVE_SET_TARGET = 300
HDMV_MSG_ID_JAW_MOVE            = 301
HDMV_MSG_ID_JAW_LAST_ID         = 399
@dataclass
class hdmv_msg_jaw_move_set_target:
    pass












