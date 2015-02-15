#!/usr/bin/env python

import rospy
import actionlib
from naoqi_msgs.msg import JointAnglesWithSpeedActionGoal
from naoqi_msgs.msg import SpeechWithFeedbackAction
from naoqi_msgs.msg import SpeechWithFeedbackActionGoal
from naoqi_msgs.srv import GetInstalledBehaviors

from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist

class RosPepperProxy(object):
    JOINT_NAMES = ['HeadYaw', 'HeadPitch',
                   'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand',
                   'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand',
                   'HipRoll', 'HipPitch', 'KneePitch']
    def __init__(self):
        self._joint_publisher = rospy.Publisher('/joint_angles_action/goal',
                                                JointAnglesWithSpeedActionGoal, queue_size=1)
        self._speech_client = actionlib.SimpleActionClient('/speech_action', SpeechWithFeedbackAction)
        self._base_move_pose_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        self._base_move_twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def move_joints(self, names, angles, speed=0.3):
        goal = JointAnglesWithSpeedActionGoal()
        goal.goal.joint_angles.joint_names = names
        goal.goal.joint_angles.joint_angles = angles
        goal.goal.joint_angles.speed = speed
        for name in names:
            if name not in self.JOINT_NAMES:
                rospy.logerr('%s is invalid joint name' % name)
                return
        self._joint_publisher.publish(goal)

    def speak(self, string, wait=False):
        self._speech_client.wait_for_server()
        goal = SpeechWithFeedbackActionGoal()
        goal.goal.say = string
        self._speech_client.send_goal(goal.goal)
        if wait:
            self._speech_client.wait_for_result()

    def get_behaviors(self):
        rospy.wait_for_service('/get_installed_behaviors')
        get_behaviors = rospy.ServiceProxy('/get_installed_behaviors', GetInstalledBehaviors)
        return get_behaviors().behaviors
        
    def move_to(self, x, y, theta):
        self._base_move_pose_publisher.publish(Pose2D(x=x, y=y, theta=theta))

    def move_vel(self, x, y, theta):
        vel = Twist()
        vel.linear.x = x
        vel.linear.y = y
        vel.angular.z = theta
        self._base_move_twist_publisher.publish(vel)


class RosPepperUtil(object):
    def __init__(self, proxy):
        self._proxy = proxy

    def move_right_hand(self, angle, speed=0.3):
        self._proxy.move_joints(['RHand'], [angle], speed=speed)

    def move_left_hand(self, angle, speed=0.3):
        self._proxy.move_joints(['LHand'], [angle], speed=speed)

    def move_head(self, yaw, pitch, speed=0.15):
        self._proxy.move_joints(['HeadYaw', 'HeadPitch'], [yaw, pitch], speed=speed)

    def move_right_arm(self, shoulder_pitch, shoulder_roll, elbow_yaw,
                       elbow_roll, wrist_yaw, speed=0.3):
        self._proxy.move_joints(
            ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw'],
            [shoulder_pitch, shoulder_roll, elbow_yaw, elbow_roll, wrist_yaw],
            speed=speed)

    def move_left_arm(self, shoulder_pitch, shoulder_roll, elbow_yaw,
                      elbow_roll, wrist_yaw, speed=0.3):
        self._proxy.move_joints(
            ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'],
            [shoulder_pitch, shoulder_roll, elbow_yaw, elbow_roll, wrist_yaw],
            speed=speed)

