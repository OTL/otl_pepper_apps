#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from std_msgs.msg import UInt8

class MyoToArm:
    def __init__(self):
        self._imu_sub = rospy.Subscriber('/myo_imu', Imu, self.get_myo_rpy)
        self._gesture_sub = rospy.Subscriber('/myo_gest', UInt8,
                                             self.get_myo_gesture)

    def get_myo_gesture(self, gesture_msg):
        if gesture_msg.data == 3:
            print 'nandeyanen!'
        elif gesture_msg.data == 4:
            print 'pa-'
        elif gesture_msg.data == 1:
            print 'gu-'

    def get_myo_rpy(self, imu_msg):
        y, p, r = euler_from_quaternion((imu_msg.orientation.w,
                                         imu_msg.orientation.x,
                                         imu_msg.orientation.y,
                                         imu_msg.orientation.z))
        # r -> shoulder-r
        # p -> shoulder-p
        #print '%f, %f, %f'%(r, p, y)


rospy.init_node('myo_arm')
m = MyoToArm()
rospy.spin()
