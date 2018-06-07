from std_msgs.msg import Float32MultiArray

FIRE_JOINT_POSITIONS = (0, -53.5, 146.5, 0, 41.2, -8.73171)
CATCH_JOINT_POSITIONS = (0, -95, 147, 0, 37.14258, -8.73171)

CATCH_JOINT_POSITIONS_MESSAGE = Float32MultiArray(data=CATCH_JOINT_POSITIONS)
FIRE_JOINT_POSITIONS_MESSAGE = Float32MultiArray(data=FIRE_JOINT_POSITIONS)

ADVERSARY_RANDOM_J1_MIN = -27
ADVERSARY_RANDOM_J1_MAX = 27

ADVERSARY_RANDOM_J5_MIN = 25
ADVERSARY_RANDOM_J5_MAX = 65

Z1_S1_FRIENDLY = {"degrees_from_center": -10, "degrees_from_45": 0, "psi": 22}
Z1_S2_FRIENDLY = {"degrees_from_center": 8, "degrees_from_45": 0, "psi": 22}

Z2_S3_FRIENDLY = {"degrees_from_center": -14, "degrees_from_45": -5, "psi": 28}
Z2_S4_FRIENDLY = {"degrees_from_center": 0, "degrees_from_45": -7, "psi": 28}
Z2_S5_FRIENDLY = {"degrees_from_center": 16, "degrees_from_45": -5, "psi": 28}

Z3_S6_FRIENDLY = {"degrees_from_center": -10, "degrees_from_45": -10, "psi": 35}
Z3_S7_FRIENDLY = {"degrees_from_center": -2, "degrees_from_45": -10, "psi": 35}
Z3_S8_FRIENDLY = {"degrees_from_center": 4, "degrees_from_45": -10, "psi": 35}
Z3_S9_FRIENDLY = {"degrees_from_center": 17, "degrees_from_45": -10, "psi": 35}

Z1_S1_ADVERSARY = {"degrees_from_center": -10, "degrees_from_45": -10, "psi": 55}
Z1_S2_ADVERSARY = {"degrees_from_center": 8, "degrees_from_45": -10, "psi": 55}

Z2_S3_ADVERSARY = {"degrees_from_center": -14, "degrees_from_45": -15, "psi": 55}
Z2_S4_ADVERSARY = {"degrees_from_center": 0, "degrees_from_45": -15, "psi": 55}
Z2_S5_ADVERSARY = {"degrees_from_center": 16, "degrees_from_45": -15, "psi": 55}

Z3_S6_ADVERSARY = {"degrees_from_center": -10, "degrees_from_45": -15, "psi": 50}
Z3_S7_ADVERSARY = {"degrees_from_center": -2, "degrees_from_45": -15, "psi": 50}
Z3_S8_ADVERSARY = {"degrees_from_center": 4, "degrees_from_45": -15, "psi": 50}
Z3_S9_ADVERSARY = {"degrees_from_center": 17, "degrees_from_45": -15, "psi": 50}
