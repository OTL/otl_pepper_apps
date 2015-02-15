#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from std_msgs.msg import UInt8
from otl_pepper_apps.pepper_utils import RosPepperProxy
from otl_pepper_apps.pepper_utils import RosPepperUtil


class MyoToArm:
    INITIAL_PITCH_ANGLE = 1.0
    INITIAL_ROLL_ANGLE = -0.2

    def __init__(self, pepper_proxy):
        self._imu_sub = rospy.Subscriber('/myo_imu', Imu, self.get_myo_rpy)
        self._gesture_sub = rospy.Subscriber('/myo_gest', UInt8,
                                             self.get_myo_gesture)
        self._pepper_proxy = pepper_proxy
        self._pepper_util = RosPepperUtil(self._pepper_proxy)
        self._r_angle = self.INITIAL_ROLL_ANGLE
        self._p_angle = self.INITIAL_PITCH_ANGLE
        self._h_angle = 0.0
        self._r_start_time = rospy.Time.now()
        self._rpy_calculator = None

    def get_myo_gesture(self, gesture_msg):
        if gesture_msg.data == 3:
            # if hand is open
            if self._h_angle > 0.0 and self._p_angle < 0.5:
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
        if self._rpy_calculator is None:
            self._rpy_calculator = RPYDiffCalculator([r, p, y])

        diff_rpy = self._rpy_calculator.get_diff_rpy([r, p, y])
        #self._r_angle = r
        self._p_angle = -diff_rpy[1] + self.INITIAL_PITCH_ANGLE
        # r -> shoulder-r
        # p -> shoulder-p
        #print '%f, %f, %f'%(r, p, y)

    def send_angles(self):
        if ((rospy.Time.now() - self._r_start_time).to_sec() > 2):
            self._r_angle = -0.2
        self._pepper_proxy.move_joints(['RShoulderPitch', 'RShoulderRoll', 'RHand'],
                                       [self._p_angle, self._r_angle, self._h_angle], speed=0.6)
        print '%f, %f, %f' % (self._p_angle, self._r_angle, self._h_angle)



class RPYDiffCalculator(object):
    def __init__(self, initial_rpy):
        self._rpy = initial_rpy

    def set_initial_rpy(self, rpy):
        self._rpy = rpy

    def get_diff_rpy(self, rpy):
        diff_rpy = [self._rpy[0] - rpy[0], self._rpy[1] - rpy[1], self._rpy[2] - rpy[2]]
        for x in diff_rpy:
            while x < -math.pi:
                x += 2 * math.pi
            while x > math.pi:
                x -= 2 * math.pi
        return diff_rpy


rospy.init_node('myo_arm')
pepper_proxy = RosPepperProxy()
pepper_proxy.speak('真似をします', True)
m = MyoToArm(pepper_proxy)
r = rospy.Rate(5.0)
while not rospy.is_shutdown():
    r.sleep()
    m.send_angles()
