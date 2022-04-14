#!/usr/bin/env python3.7

# encoding: utf-8

"""Module to connect to a kinect through ROS and record stream
"""

## Need this to be running for camera: roslaunch openni_launch openni.launch

from __future__ import print_function
import rospy
import tf
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
import time
import datetime
import tf2_ros
import os
import pyautogui
import numpy as np
import mss
import mss.tools


os.chdir(os.path.expanduser("~/catkin_ws/src/visuoTactileHRC/user_trial_data_recording/"))

test_mode = False
recording = True
kin_image = None
timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
file = f"hrc_output_{timestamp}.avi"
task = 0
user = None
cmd = None
camera_info = None


class image_converter:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)
    #self.image_sub = rospy.Subscriber("/camera/depth/image",Image,self.callback)

  def callback(self, data):
    global kin_image
    try:
      #cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
      kin_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv2.putText(kin_image, str(time.time()), (350, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
    except CvBridgeError as e:
      print(e)


def image_info_cb(data):
    global camera_info
    camera_info = data


def cmd_callback(msg):
    global recording, user, task, cmd, file
    if msg.data != cmd:
	    cmd = msg.data
	    if cmd == 'start':
	        print("video recording starting")
	        timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
	        file = f"hrc_output_{time.time()}_user_{user}_take_{task}_{timestamp}.avi"
	        recording = True
	    elif cmd == 'stop':
	        recording = False
	        print("video recording stopping")
	    # elif cmd == 'Discard':
	    #     data = []
	    #     print("Data discarded")
	    # elif cmd == 'Save':
	    #     save_data()
	    elif 'User' in cmd:
	        user = cmd.split("User:", 1)[1]
	        print(f"video new user: {user}")
	    elif 'Task' in cmd:
	        task = cmd.split("Task:", 1)[1]
	        print(f"video new take: {task}")
	    elif cmd == 'Quit':
	    	rospy.signal_shutdown('video quit cmd received')


def main(args):
    global kin_image, file
    rospy.init_node('skeleton_viewer', anonymous=True)

    image_info_sub = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, image_info_cb)
    rospy.Subscriber('ProcessCommands', String, cmd_callback)

    if test_mode:
        cap= cv2.VideoCapture(0)

        size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        	int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    else:
        ic = image_converter()

        cam_model = PinholeCameraModel()
        while (not camera_info) and (not rospy.is_shutdown()):
            time.sleep(1)
            print("video recorder waiting for camera info")

        cam_model.fromCameraInfo(camera_info)
        size = cam_model.fullResolution()
        print(cam_model.cx(), cam_model.cy(), cam_model.fullResolution())

    fps = 30
    rate = rospy.Rate(fps)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    kinect_writer = cv2.VideoWriter()
    k_writer_open = False
    s_writer_open = False
    SCREEN_SIZE = tuple(pyautogui.size())
    screen_writer = cv2.VideoWriter()

    with mss.mss() as sct:
        # The screen part to capture
        region = {'top': 0, 'left': 0, 'width': SCREEN_SIZE[0], 'height': SCREEN_SIZE[1]}

        while not rospy.is_shutdown():
            if test_mode:
                ret, kin_image= cap.read()

            # Grab the screenshot data
            screen_img = sct.grab(region)
            screen_frame = cv2.cvtColor(np.array(screen_img), cv2.COLOR_RGBA2RGB)

            if recording:
                if not k_writer_open:
                    k_writer_open = kinect_writer.open("kinect_"+file, fourcc, fps, size, True)
                if not s_writer_open:
                    s_writer_open = screen_writer.open("screen_"+file, fourcc, fps, SCREEN_SIZE, True)

                kinect_writer.write(kin_image)
                cv2.putText(screen_frame, str(time.time()), (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                screen_writer.write(screen_frame)

            else:
                if k_writer_open:
                    kinect_writer.release()
                    k_writer_open = False
                if s_writer_open:
                    screen_writer.release()
                    s_writer_open = False

            cv2.imshow("Image window", kin_image)

            if cv2.waitKey(1) & 0xFF == 27:
                break

            rate.sleep()

        if test_mode:
            cap.release()

        if k_writer_open:
            kinect_writer.release()
        if s_writer_open:
            screen_writer.release()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
