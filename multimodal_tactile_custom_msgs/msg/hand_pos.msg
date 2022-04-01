## Message containing location of a users hand
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
## Which hand location is for, 0:left, 1:right
int8 Hand
#
# geometry_msgs/Pose.msg
# A representation of pose in free space, composed of position and orientation. 
# Point position
# Quaternion orientation
geometry_msgs/Pose Pose