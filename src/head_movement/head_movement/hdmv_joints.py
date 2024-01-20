# head joints are defined in /config/dynamixel_head.yaml

# joint ids
joint_id_head_pan_joint             = 4
joint_id_head_tilt_right_joint      = 1
joint_id_head_tilt_left_joint       = 3
joint_id_head_tilt_vertical_joint   = 2
joint_id_eye_shift_horizontal_joint = 9
joint_id_eye_shift_vertical_joint   = 11
joint_id_jaw_joint                  = 12

# default joint values
joint_defval_head_pan_joint             = 0.6
joint_defval_head_tilt_right_joint      = 0.5
joint_defval_head_tilt_left_joint       = -0.5
joint_defval_head_tilt_vertical_joint   = 1.2
joint_defval_eye_shift_horizontal_joint = -0.7
joint_defval_eye_shift_vertical_joint   = -0.75
joint_defval_jaw_joint                  = -1

joint_id_to_name = {
    joint_id_head_pan_joint             : "joint_id_head_pan_joint",             \
    joint_id_head_tilt_right_joint      : "joint_id_head_tilt_right_joint",      \
    joint_id_head_tilt_left_joint       : "joint_id_head_tilt_left_joint",       \
    joint_id_head_tilt_vertical_joint   : "joint_id_head_tilt_vertical_joint",   \
    joint_id_eye_shift_horizontal_joint : "joint_id_eye_shift_horizontal_joint", \
    joint_id_eye_shift_vertical_joint   : "joint_id_eye_shift_vertical_joint",   \
    joint_id_jaw_joint                  =  "joint_id_jaw_joint" \
    }

joint_id_to_defval = {
    joint_id_head_pan_joint             : joint_defval_head_pan_joint,             \
    joint_id_head_tilt_right_joint      : joint_defval_head_tilt_right_joint,      \
    joint_id_head_tilt_left_joint       : joint_defval_head_tilt_left_joint,       \
    joint_id_head_tilt_vertical_joint   : joint_defval_head_tilt_vertical_joint,   \
    joint_id_eye_shift_horizontal_joint : joint_defval_eye_shift_horizontal_joint, \
    joint_id_eye_shift_vertical_joint   : joint_defval_eye_shift_vertical_joint,   \
    joint_id_jaw_joint                  = joint_defval_jaw_joint \
    }
