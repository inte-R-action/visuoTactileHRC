#!/usr/bin/env python3

import rospy
from sam_custom_messages.msg import object_state
from std_msgs.msg import String


class screw_counter():
    def __init__(self):
        rospy.Subscriber('ObjectStates', object_state, self.vision_callback)
        self.publisher = rospy.Publisher('ScrewCount', String, queue_size=10)
        self.seq = None
        self.sides = []
        self.screws = []

    def vision_callback(self, data):          
        if data.Header.seq == self.seq:
            if data.Object.Info[0] == 'screw':
                self.screws.append([data.Pose.position.x, data.Pose.position.y])
            elif data.Object.Info[0] in ['box_base', 'box_side_big', 'box_side_small']:
                self.sides.append([data.Pose.orientation.x, data.Pose.orientation.y, data.Pose.orientation.z, data.Pose.orientation.w, data.Object.Info[0]])
        else:
            self.count_screws()
            self.seq = data.Header.seq
            self.sides = []
            self.screws = []

    def count_screws(self):
        for side in self.sides:
            screw_count = 0
            for screw in self.screws:
                if (screw[0] > side[0]) and (screw[0] < side[2]):
                    if (screw[1] > side[1]) and (screw[1] < side[3]):
                        screw_count += 1

            print(f"{side[-1]} {screw_count}")
            self.publisher.publish(f"{side[-1]} {screw_count}")
        

def run():
    # ROS node setup
    rospy.init_node(f'screw_counter', anonymous=True)
    frame_id = 'screw_counter'

    counter = screw_counter()    

    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        print(f'Screw counter running')
        rate.sleep()

if __name__ == '__main__':
    run()
    
