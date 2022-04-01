## Message containing id, joint states and end effector position of robot
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
# sensor_msgs/JointState.msg
# This is a message that holds data to describe the state of a set of torque controlled joints. 
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.
# Header header
# string[] name
# float64[] position
# float64[] velocity
# float64[] effort
sensor_msgs/JointState JointState
#
# geometry_msgs/Pose.msg
# A representation of end effector pose in free space, composed of position and orientation. 
# Point position
# Quaternion orientation
geometry_msgs/Pose Pose