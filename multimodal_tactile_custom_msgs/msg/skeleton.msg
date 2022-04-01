## Message containing skeleton points for a user
#
## std_msgs/Header.msg
# sequence ID: consecutively increasing ID 
# uint32 seq
# Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
# time stamp
# Frame this data is associated with
# string frame_id
Header Header
#
## ID of user (robot or human)
int8 UserId
#
## Name of user (robot or human)
string UserName
#
# geometry_msgs/Pose.msg
# A representation of pose in free space, composed of position and orientation. 
# Point position
# Quaternion orientation
geometry_msgs/Pose head
geometry_msgs/Pose neck
geometry_msgs/Pose torso
geometry_msgs/Pose left_shoulder
geometry_msgs/Pose left_elbow
geometry_msgs/Pose left_hand
geometry_msgs/Pose left_hip
geometry_msgs/Pose left_knee
geometry_msgs/Pose left_foot
geometry_msgs/Pose right_shoulder
geometry_msgs/Pose right_elbow
geometry_msgs/Pose right_hand
geometry_msgs/Pose right_hip
geometry_msgs/Pose right_knee
geometry_msgs/Pose right_foot