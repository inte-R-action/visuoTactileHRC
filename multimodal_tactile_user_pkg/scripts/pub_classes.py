#!/usr/bin/env python3.7

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Accel, Vector3
from multimodal_tactile_custom_msgs.msg import object_state, diagnostics, current_action, robot_move, user_prediction, capability, screw_count, threeIMUs
from diagnostic_msgs.msg import KeyValue


class diag_class:
    def __init__(self, frame_id, user_id=1, user_name="unknown", queue=1, keyvalues=[]):
        # frame_id=str, user_id=int, user_name=str, queue=int, keyvalues=list of KeyValue items
        # Diagnostic message definitions
        self.diag_msg = diagnostics()
        self.diag_msg.Header.stamp = rospy.get_rostime()
        self.diag_msg.Header.seq = None
        self.diag_msg.Header.frame_id = frame_id
        self.diag_msg.UserId = user_id
        self.diag_msg.UserName = user_name
        self.diag_msg.DiagnosticStatus.level = 1 # 0:ok, 1:warning, 2:error, 3:stale
        self.diag_msg.DiagnosticStatus.name = frame_id
        self.diag_msg.DiagnosticStatus.message = "Starting..."
        self.diag_msg.DiagnosticStatus.hardware_id = "N/A"
        self.diag_msg.DiagnosticStatus.values = keyvalues

        self.publisher = rospy.Publisher('SystemStatus', diagnostics, queue_size=queue)
        self.publish(1, "Starting...")

    def publish(self, level, message, keyvalues=[]):
        self.diag_msg.DiagnosticStatus.level = level # 0:ok, 1:warning, 2:error, 3:stale
        self.diag_msg.DiagnosticStatus.message = message
        self.diag_msg.DiagnosticStatus.values = keyvalues
        self.diag_msg.Header.stamp = rospy.get_rostime()
        if self.diag_msg.Header.seq is None:
            self.diag_msg.Header.seq = 0
        else:
            self.diag_msg.Header.seq += 1

        self.publisher.publish(self.diag_msg)

class obj_class:
    def __init__(self, frame_id, names, queue=1):
        # frame_id=str, names=pytorch classifer output names, queue=int
        # Object message definitions
        self.obj_msg = object_state()
        self.obj_msg.Header.stamp = rospy.get_rostime()
        self.obj_msg.Header.seq = None
        self.obj_msg.Header.frame_id = frame_id
        self.obj_msg.Object.Id = None
        self.obj_msg.Object.Obj_type = None
        self.obj_msg.Object.Info = None
        self.obj_msg.Pose.orientation.x = None
        self.obj_msg.Pose.orientation.y = None
        self.obj_msg.Pose.orientation.z = None
        self.obj_msg.Pose.orientation.w = None
        self.obj_msg.Pose.position.x = None
        self.obj_msg.Pose.position.y = None
        self.obj_msg.Pose.position.z = None
        self.names = names

        self.publisher = rospy.Publisher('ObjectStates', object_state, queue_size=queue)

    def publish(self, det, msg_type):
        if msg_type == "classifier":
            # cnn classifier method type
            # det=object for items in image?
            if self.obj_msg.Header.seq is None:
                self.obj_msg.Header.seq = 0
            else:
                self.obj_msg.Header.seq += 1
            
            for *xyxy, conf, cls, dist in det:
                self.obj_msg.Object.Id = 0
                self.obj_msg.Object.Obj_type = int(cls)
                self.obj_msg.Object.Info = [self.names[int(cls)]]
                self.obj_msg.Pose.orientation.x = xyxy[0]
                self.obj_msg.Pose.orientation.y = xyxy[1]
                self.obj_msg.Pose.orientation.z = xyxy[2]
                self.obj_msg.Pose.orientation.w = xyxy[3]
                self.obj_msg.Pose.position.x = (xyxy[0]+xyxy[2])/2
                self.obj_msg.Pose.position.y = (xyxy[1]+xyxy[3])/2
                self.obj_msg.Pose.position.z = dist
                self.obj_msg.Header.stamp = rospy.get_rostime()

                self.publisher.publish(self.obj_msg)

        elif msg_type == "dip":
            #Digital impage processing method type
            if self.obj_msg.Header.seq is None:
                self.obj_msg.Header.seq = 0
            else:
                self.obj_msg.Header.seq += 1
            
            self.obj_msg.Object.Id = 0
            self.obj_msg.Object.Obj_type = int(det[1])
            self.obj_msg.Object.Info = det[2]
            self.obj_msg.Pose = det[0]
            self.obj_msg.Header.stamp = rospy.get_rostime()

            self.publisher.publish(self.obj_msg)

class act_class:
    def __init__(self, frame_id, class_count, user_id=1, user_name="unknown", queue=1):
        # frame_id=str, class_count=int, user_id=int, user_name=str, queue=int
        # Current action message definitions
        self.act_msg = current_action()
        self.act_msg.Header.stamp = rospy.get_rostime()
        self.act_msg.Header.seq = None
        self.act_msg.Header.frame_id = frame_id
        self.act_msg.UserId = user_id
        self.act_msg.UserName = user_name
        self.act_msg.ActionProbs = [0]*class_count

        self.publisher = rospy.Publisher('CurrentAction', current_action, queue_size=queue)

    def publish(self, prediction):
        # prediction=list floats
        if self.act_msg.Header.seq is None:
            self.act_msg.Header.seq = 0
        else:
            self.act_msg.Header.seq += 1
        
        self.act_msg.ActionProbs = prediction
        self.act_msg.Header.stamp = rospy.get_rostime()

        self.publisher.publish(self.act_msg)

class move_class:
    def __init__(self, frame_id, queue=10):
        # frame_id=str, queue=int
        # Current action message definitions
        #self.move_msg = robot_move()
        #self.move_msg.Header.stamp = rospy.get_rostime()
        #self.move_msg.Header.seq = None
        #self.move_msg.Header.frame_id = frame_id

        self.publisher = rospy.Publisher('RobotMove', String, queue_size=queue)

    def publish(self, command):
        # command = str()
        # if self.move_msg.Header.seq is None:
        #     self.move_msg.Header.seq = 0
        # else:
        #     self.move_msg.Header.seq += 1

        #self.move_msg.Command = commands
        #self.move_msg.Header.stamp = rospy.get_rostime()

        #self.msg = command

        self.publisher.publish(command)

class future_class:
    def __init__(self, frame_id, user_id=1, queue=1):
        # frame_id=str, queue=int
        # Current action message definitions
        self.future_msg = user_prediction()
        self.future_msg.Header.stamp = rospy.get_rostime()
        self.future_msg.Header.seq = None
        self.future_msg.Header.frame_id = frame_id
        self.future_msg.UserId = user_id
        self.future_msg.Predictions = []

        self.publisher = rospy.Publisher('FutureState', user_prediction, queue_size=queue)

    def publish(self, prediction):
        # prediction = float64[]
        if self.future_msg.Header.seq is None:
            self.future_msg.Header.seq = 0
        else:
            self.future_msg.Header.seq += 1
        
        self.future_msg.Command = prediction
        self.future_msg.Header.stamp = rospy.get_rostime()

        self.publisher.publish(self.future_msg)

class capability_class:
    def __init__(self, frame_id, user_id=1, queue=1):
        # frame_id=str, queue=int
        # Current action message definitions
        self.capability_msg = capability()
        self.capability_msg.Header.stamp = rospy.get_rostime()
        self.capability_msg.Header.seq = None
        self.capability_msg.Header.frame_id = frame_id
        self.capability_msg.UserId = user_id
        self.capability_msg.Object.Id = 0
        self.capability_msg.Object.Type = 0
        self.capability_msg.Object.Info = "N/A"
        self.capability_msg.Pose.orientation.x = 0
        self.capability_msg.Pose.orientation.y = 0
        self.capability_msg.Pose.orientation.z = 0
        self.capability_msg.Pose.orientation.w = 0
        self.capability_msg.Pose.position.x = 0
        self.capability_msg.Pose.position.y = 0
        self.capability_msg.Pose.position.z = 0
        self.capability_msg.Type = 0
        self.capability_msg.Info = []

        self.publisher = rospy.Publisher('CurrentState', capability, queue_size=queue)

    def publish(self, cap_type, cap_info):
        # prediction = float64[]
        if self.capability_msg.Header.seq is None:
            self.capability_msg.Header.seq = 0
        else:
            self.capability_msg.Header.seq += 1
        
        self.capability_msg.Type = cap_type
        self.capability_msg.Info = cap_info
        self.capability_msg.Header.stamp = rospy.get_rostime()

        self.publisher.publish(self.capability_msg)

class threeIMUs_class:
    def __init__(self, frame_id, user_id=1, user_name='unknown', queue=1):
        # frame_id=str, queue=int
        # Current IMU data message definitions
        self.IMUs_msg = threeIMUs()
        self.IMUs_msg.Header.stamp = rospy.get_rostime()
        self.IMUs_msg.Header.seq = None
        self.IMUs_msg.Header.frame_id = frame_id
        self.IMUs_msg.UserId = user_id
        self.IMUs_msg.UserName = user_name
        self.accel_msg = Accel()
        self.positions = ['Hand', 'Wrist', 'Arm']

        for p in range(len(self.positions)):
            self.accel_msg.linear = Vector3(0,0,0)
            self.accel_msg.angular = Vector3(0,0,0)
            setattr(self.IMUs_msg, self.positions[p], self.accel_msg)

        self.publisher = rospy.Publisher('IMUdata', threeIMUs, queue_size=queue)

    def publish(self, IMU_data):
    #     IMU_data: [hand accel_x, hand accel_y, hand accel_z,
    #               hand gyro_x, hand gyro_y, hand gyro_z,
    #               wrist accel_x, wrist accel_y, wrist accel_z,
    #               wrist gyro_x, wrist gyro_y, wrist gyro_z,
    #               arm accel_x, arm accel_y, arm accel_z,
    #               arm gyro_x, arm gyro_y, arm gyro_z]
        if self.IMUs_msg.Header.seq is None:
            self.IMUs_msg.Header.seq = 0
        else:
            self.IMUs_msg.Header.seq += 1

        for p in range(len(self.positions)):
            self.accel_msg.linear = Vector3(float(IMU_data[p*6]), float(IMU_data[(p*6)+1]), float(IMU_data[(p*6)+2]))
            self.accel_msg.angular = Vector3(float(IMU_data[(p*6)+3]), float(IMU_data[(p*6)+4]), float(IMU_data[(p*6)+5]))
            setattr(self.IMUs_msg, self.positions[p], self.accel_msg)

        self.IMUs_msg.Header.stamp = rospy.get_rostime()
        self.publisher.publish(self.IMUs_msg)

class screw_count_class:
    def __init__(self, frame_id, user_id=1, user_name="unknown", queue=1):
        # frame_id=str, user_id=int, user_name=str, queue=int
        # Screw counts message definitions
        self.screw_msg = screw_count()
        self.screw_msg.Header.stamp = rospy.get_rostime()
        self.screw_msg.Header.seq = None
        self.screw_msg.Header.frame_id = frame_id
        self.screw_msg.UserId = user_id
        self.screw_msg.UserName = user_name
        self.screw_msg.ScrewCount = 0
        self.screw_msg.LastScrewCount = 0

        self.publisher = rospy.Publisher('ScrewCounts', screw_count, queue_size=queue)

    def publish(self, count_now, count_last):
        # count_now=int, count_last=int
        if self.screw_msg.Header.seq is None:
            self.screw_msg.Header.seq = 0
        else:
            self.screw_msg.Header.seq += 1
        
        self.screw_msg.ScrewCount = count_now
        self.screw_msg.LastScrewCount = count_last
        self.screw_msg.Header.stamp = rospy.get_rostime()

        self.publisher.publish(self.screw_msg)