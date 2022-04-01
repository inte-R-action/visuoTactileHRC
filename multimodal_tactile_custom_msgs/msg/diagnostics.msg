## Message containing diagnostic information for sensors/devices/users within the system
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
## diagnostic_msgs/DiagnosticStatus.msg
# Possible levels of operations
# byte OK=0
# byte WARN=1
# byte ERROR=2
# byte STALE=3
# byte level # level of operation enumerated above 
# string name # a description of the test/component reporting
# string message # a description of the status
# string hardware_id # a hardware unique string
# KeyValue[] values # an array of values associated with the status
diagnostic_msgs/DiagnosticStatus DiagnosticStatus