#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from std_msgs.msg import UInt8
from otl_pepper_apps.pepper_utils import RosPepperProxy
from otl_pepper_apps.pepper_utils import RosPepperUtil


class MyoToArm:
    def __init__(self):
        self._imu_sub = rospy.Subscriber('/myo_imu', Imu, self.get_myo_rpy)
        self._gesture_sub = rospy.Subscriber('/myo_gest', UInt8,
                                             self.get_myo_gesture)
        self._pepper_proxy = RosPepperProxy()
        self._pepper_util = RosPepperUtil(self._pepper_proxy)
        self._r_angle = -0.2
        self._p_angle = 1.0
        self._h_angle = 0.0
        self._r_start_time = rospy.Time.now()

    def get_myo_gesture(self, gesture_msg):
        if gesture_msg.data == 3:
            # if hand is open
            if self._h_angle > 0.0:
                self._pepper_proxy.speak('なんでやねん')
                self._r_angle = -1.5
                self._r_start_time = rospy.Time.now()
        elif gesture_msg.data == 4:
            self._h_angle = 1.0
        elif gesture_msg.data == 1:
            self._h_angle = 0.0

    def get_myo_rpy(self, imu_msg):
        y, p, r = euler_from_quaternion((imu_msg.orientation.w,
                                         imu_msg.orientation.x,
                                         imu_msg.orientation.y,
                                         imu_msg.orientation.z))
        #self._r_angle = r
        self._p_angle = p
        # r -> shoulder-r
        # p -> shoulder-p
        #print '%f, %f, %f'%(r, p, y)

    def send_angles(self):
        if ((rospy.Time.now() - self._r_start_time).to_sec() > 2):
            self._r_angle = -0.2
        self._pepper_proxy.move_joints(['RShoulderPitch', 'RShoulderRoll', 'RHand'],
                                       [self._p_angle, self._r_angle, self._h_angle], speed=0.6)
        print '%f, %f, %f' % (self._p_angle, self._r_angle, self._h_angle)


rospy.init_node('myo_arm')
m = MyoToArm()
r = rospy.Rate(10.0)
while not rospy.is_shutdown():
    r.sleep()
    m.send_angles()

    
