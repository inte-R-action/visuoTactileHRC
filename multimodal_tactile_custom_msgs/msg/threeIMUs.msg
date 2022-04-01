## Message containing IMU data from 3 sensors on user
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
# geometry_msgs/Accel.msg
# This expresses acceleration in free space broken into its linear and angular parts.
# Vector3  linear => float64 x, float64 y, float64 z
# Vector3  angular => float64 x, float64 y, float64 z
geometry_msgs/Accel Hand
geometry_msgs/Accel Wrist
geometry_msgs/Accel Arm
