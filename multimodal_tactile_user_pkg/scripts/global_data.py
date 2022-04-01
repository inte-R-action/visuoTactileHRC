ALL_ACTIONS = ["Null", "Screw In", "Screw Out", "Allen In", "Allen Out", "Hammer", "Hand Screw In", "Hand Screw Out", "Wave", "Forward", "Backward", "Left", "Right", "Stop"]

COMPLEX_BOX_ACTIONS = ["null", "screw_in", "allen_in", "hammer", "hand_screw_in"]
ACTIONS = ["screw_in", "allen_in", "hammer", "hand_screw_in"]
num_chair_actions = {"screw_in": 4, "allen_in": 2, "hammer": 4, "hand_screw_in": 4}
num_box_actions = {"screw_in": 12, "allen_in": 4, "hammer": 4, "hand_screw_in": 2}
TASKS = ['assemble_complex_box', 'assemble_complex_box_manual', 'assemble_chair']
DEFAULT_TASK = TASKS[0]
inclAdjParam = True

SKELETON_FRAMES = [
    'head',
    'neck',
    'torso',
    'left_shoulder',
    'left_elbow',
    'left_hand',
    'left_hip',
    'left_knee',
    'left_foot',
    'right_shoulder',
    'right_elbow',
    'right_hand',
    'right_hip',
    'right_knee',
    'right_foot',
]

JOINTS_U = [
    'head',
    'neck',
    'torso',
    'left_shoulder',
    'left_elbow',
    'left_hand',
    'left_hip',
    'right_shoulder',
    'right_elbow',
    'right_hand',
    'right_hip',
    ]

JOINT_LINKS = [[0, 1, 1, 3, 3, 4, 2, 2, 2, 7, 8],
               [1, 3, 7, 2, 4, 5, 6, 7, 10, 8, 9]]

PCA_COMPS = 20
