// Map high level position to joint angles as seen on teach pendant
// Order is: shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint
struct jnt_angs{double angles[6];};
std::map<std::string, jnt_angs> joint_positions;
joint_positions["home"] = {-11.75, -83.80, 47.90, -125.0, -90.0, 0.0};
joint_positions["bring_side_1"] = {54.1, -75.9, 49.1, -61.2, -90.0, 0.0};
joint_positions["bring_side_2"] = {80.9, -75.9, 49.1, -61.2, -90.0, 0.0};
joint_positions["bring_side_3"] = {119.3, -75.9, 49.1, -61.2, -90.0, 0.0};
joint_positions["bring_side_4"] = {154.5, -75.9, 49.1, -61.2, -90.0, 0.0};
joint_positions["take_box"] = {14.5, -45.90, 12.2, -61.2, -91.1, 0.0};
joint_positions["deliver_2_user"] = {14.5, -45.90, 12.2, -61.2, -90.0, 0.0};
joint_positions["deliver_big_2_user"] = {17.4, -68.3, 35.9, -60.5, -91.0, 0.0};
joint_positions["deliver_box"] = {130.0, -62.90, 40.2, -61.2, -91.1, 0.0};
joint_positions["stack_red_small_block"] = {-110.0, -75.9, 49.1, -61.2, -90.0, 0.0};
joint_positions["stack_blue_small_block"] = {-130.0, -75.9, 49.1, -61.2, -90.0, 0.0};
joint_positions["stack_green_small_block"] = {-150.0, -75.9, 49.1, -61.2, -90.0, 0.0};
joint_positions["stack_yellow_small_block"] = {-90.0, -75.9, 49.1, -61.2, -90.0, 0.0};
joint_positions["look_for_objects"] = {-110.0, -75.9, 49.1, -61.2, -90.0, 0.0};
joint_positions["final_stack"] = {-20.0, -75.9, 49.1, -61.2, -91.1, 0.0};
joint_positions["remove_stack"] = {-170.0, -75.9, 49.1, -61.2, -91.1, 0.0};
joint_positions["bring_hand_screw_parts"] = {-65.0, -75.9, 49.1, -61.2, -90.0, 0.0};
joint_positions["bring_seat_top"] = {54.1, -79.0, 41.0, -52.4, -90, 0.0};
joint_positions["bring_back_frame"] = {80.9, -79.0, 41.0, -52.4, -90, 0.0};
joint_positions["bring_back_slats"] = {119.3, -75.9, 49.1, -61.2, -90, 0.0};
joint_positions["bring_legs"] = {-65.0, -75.9, 49.1, -61.2, -90, 0.0};
joint_positions["take_chair"] = {17.4, -68.3, 35.9, -60.5, -90, 0.0};
joint_positions["deliver_chair"] = {130.0, -63.8, 20.1, -46.8, -90, 0.0};