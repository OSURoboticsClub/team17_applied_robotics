from std_msgs.msg import Float32MultiArray

FIRE_JOINT_POSITIONS = (0, -53.5, 146.5, 0, 41.2, -8.73171)
CATCH_JOINT_POSITIONS = (0, -95, 147, 0, 37.14258, -8.73171)

CATCH_JOINT_POSITIONS_MESSAGE = Float32MultiArray(data=CATCH_JOINT_POSITIONS)
FIRE_JOINT_POSITIONS_MESSAGE = Float32MultiArray(data=FIRE_JOINT_POSITIONS)

ADVERSARY_RANDOM_J1_MIN = -30
ADVERSARY_RANDOM_J1_MAX = 30

ADVERSARY_RANDOM_J5_MIN = 20
ADVERSARY_RANDOM_J5_MAX = 45

Z1_S1_FRIENDLY = {"degrees_from_center": -17, "degrees_from_45": 0, "psi": 10}
Z1_S2_FRIENDLY = {"degrees_from_center": 17, "degrees_from_45": 0, "psi": 10}

Z2_S3_FRIENDLY = {"degrees_from_center": -17, "degrees_from_45": 0, "psi": 15}
Z2_S4_FRIENDLY = {"degrees_from_center": 0, "degrees_from_45": 0, "psi": 15}
Z2_S5_FRIENDLY = {"degrees_from_center": 17, "degrees_from_45": 0, "psi": 15}

Z3_S6_FRIENDLY = {"degrees_from_center": -17, "degrees_from_45": 0, "psi": 20}
Z3_S7_FRIENDLY = {"degrees_from_center": -4.5, "degrees_from_45": 0, "psi": 20}
Z3_S8_FRIENDLY = {"degrees_from_center": 4.5, "degrees_from_45": 0, "psi": 20}
Z3_S9_FRIENDLY = {"degrees_from_center": 17, "degrees_from_45": 0, "psi": 20}

Z1_S1_ADVERSARY = {"degrees_from_center": -17, "degrees_from_45": 0, "psi": 30}
Z1_S2_ADVERSARY = {"degrees_from_center": 17, "degrees_from_45": 0, "psi": 30}

Z2_S3_ADVERSARY = {"degrees_from_center": -17, "degrees_from_45": 0, "psi": 30}
Z2_S4_ADVERSARY = {"degrees_from_center": 0, "degrees_from_45": 0, "psi": 30}
Z2_S5_ADVERSARY = {"degrees_from_center": 17, "degrees_from_45": 0, "psi": 30}

Z3_S6_ADVERSARY = {"degrees_from_center": -17, "degrees_from_45": 0, "psi": 30}
Z3_S7_ADVERSARY = {"degrees_from_center": -4.5, "degrees_from_45": 0, "psi": 30}
Z3_S8_ADVERSARY = {"degrees_from_center": 4.5, "degrees_from_45": 0, "psi": 30}
Z3_S9_ADVERSARY = {"degrees_from_center": 17, "degrees_from_45": 0, "psi": 30}
