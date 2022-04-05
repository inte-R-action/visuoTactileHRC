ALL_ACTIONS = ["Null", "Screw In", "Screw Out", "Allen In", "Allen Out", "Hammer", "Hand Screw In", "Hand Screw Out", "Wave", "Forward", "Backward", "Left", "Right", "Stop"]

ACTIONS = ["screw_in", "allen_in", "hammer", "hand_screw_in"]
ACTIONS_NULL = ["null", "screw_in", "allen_in", "hammer", "hand_screw_in"]
GESTURES =  ["Null", "Wave", "Forward", "Backward", "Left", "Right", "Stop"]
num_chair_actions = {"screw_in": 4, "allen_in": 2, "hammer": 4, "hand_screw_in": 4}
num_box_actions = {"screw_in": 12, "allen_in": 4, "hammer": 4, "hand_screw_in": 2}
TASKS = ['assemble_complex_box', 'assemble_complex_box_manual', 'assemble_chair']
DEFAULT_TASK = TASKS[0]
