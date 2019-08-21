#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
# Name: move_crab.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 03/27/2018
# Edit Date: 03/27/2018
#
# Description:
#   4-wheel drive, skid-steer move_base script.
"""


from __future__ import print_function

import math as m
import copy

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from hebiros.srv import AddGroupFromNamesSrv, SendCommandWithAcknowledgementSrv
from hebiros.msg import CommandMsg

from common.pid_controller import PIDController as pid

WHEELS = 4


def round_list(lst, num_decimal):
    return [round(num, num_decimal) for num in lst]


def clamp(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n


def check_if_twist_is_zero(twist, linear_threshold, angular_threshold):
    assert isinstance(twist, Twist)
    if abs(twist.linear.x) > linear_threshold:
        return False
    elif abs(twist.linear.y) > linear_threshold:
        return False
    elif abs(twist.linear.z) > linear_threshold:
        return False
    elif abs(twist.angular.x) > angular_threshold:
        return False
    elif abs(twist.angular.y) > angular_threshold:
        return False
    elif abs(twist.angular.z) > angular_threshold:
        return False
    else:
        return True


class SkidSteer(object):
    """
        hebi_group_name     (str):
        hebi_mapping        (dict of str): {"fl":  ,"fr":  ,"bl":  ,"br"}
    """
    def __init__(self, hebi_group_name, hebi_mapping):
        rospy.loginfo("Creating SkidSteer instance...")
        hebi_families = []
        hebi_names = []
        for k,v in hebi_mapping.iteritems():
            family, name = v.split('/')
            hebi_families.append(family)
            hebi_names.append(name)
        rospy.loginfo("  hebi_group_name: %s", hebi_group_name)
        rospy.loginfo("  hebi_families: %s", hebi_families)
        rospy.loginfo("  hebi_names: %s", hebi_names)
        self.hebi_mapping = [motor for key,motor in hebi_mapping.iteritems()]
        self.hebi_mapping_dict = hebi_mapping

        # jt information populated by self._feedback_cb
        self._current_jt_pos = {}
        self._current_jt_vel = {}
        self._current_jt_eff = {}
        self._joint_state_pub = None

        # Create a service client to create a group
        set_hebi_group = rospy.ServiceProxy('/hebiros/add_group_from_names', AddGroupFromNamesSrv)
        # Topic to receive feedback from a group
        self.hebi_group_feedback_topic = "/hebiros/"+hebi_group_name+"/feedback/joint_state"
        rospy.loginfo("  hebi_group_feedback_topic: %s", "/hebiros/"+hebi_group_name+"/feedback/joint_state")
        # Topic to send commands to a group
        self.hebi_group_command_topic = "/hebiros/"+hebi_group_name+"/command/joint_state"
        rospy.loginfo("  hebi_group_command_topic: %s", "/hebiros/"+hebi_group_name+"/command/joint_state")
        # Call the /hebiros/add_group_from_names service to create a group
        rospy.loginfo("  Waiting for AddGroupFromNamesSrv at %s ...", '/hebiros/add_group_from_names')
        rospy.wait_for_service('/hebiros/add_group_from_names')  # block until service server starts
        rospy.loginfo("  AddGroupFromNamesSrv AVAILABLE.")
        set_hebi_group(hebi_group_name, hebi_names, hebi_families)
        # Create a service client to set group settings
        change_group_settings = rospy.ServiceProxy("/hebiros/"+hebi_group_name+"/send_command_with_acknowledgement",
                                                   SendCommandWithAcknowledgementSrv)
        rospy.loginfo("  Waiting for SendCommandWithAcknowledgementSrv at %s ...", "/hebiros/"+hebi_group_name+"/send_command_with_acknowledgement")
        rospy.wait_for_service("/hebiros/"+hebi_group_name+"/send_command_with_acknowledgement")  # block until service server starts
        cmd_msg = CommandMsg()
        cmd_msg.name = self.hebi_mapping
        cmd_msg.settings.name = cmd_msg.name
        cmd_msg.settings.velocity_gains.name = cmd_msg.name
        cmd_msg.settings.position_gains.kp = [10]*WHEELS  # FIXME: Tune
        cmd_msg.settings.position_gains.ki = [5]*WHEELS  # FIXME: Tune
        cmd_msg.settings.position_gains.kd = [0.1]*WHEELS  # FIXME: Tune
        cmd_msg.settings.position_gains.i_clamp = [0.1]*WHEELS  # FIXME: Tune
        change_group_settings(cmd_msg)
        # Feedback/Command
        self.fbk_sub = rospy.Subscriber(self.hebi_group_feedback_topic, JointState, self._feedback_cb)
        self.cmd_pub = rospy.Publisher(self.hebi_group_command_topic, JointState, queue_size=1)

        # Twist Subscriber
        self._cmd_vel_sub = rospy.Subscriber("/skidsteer/cmd_vel/", Twist, self._cmd_vel_cb)
        self.last_vel_cmd = None
        self.linear_displacement_limit = 0.075  # m
        self.angular_displacement_limit = 0.65  # rad

        # Wait for connections to be set up
        rospy.loginfo("Wait for ROS connections to be set up...")
        while not rospy.is_shutdown() and len(self._current_jt_pos) < len(self.hebi_mapping):
            rospy.sleep(0.1)
        self._joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

        self._loop_rate = rospy.Rate(200)

        self.lin_vel = 7.5
        self.ang_vel = 7.5

        rospy.loginfo("Done creating SkidSteer instance...")

    def loop(self):
        rospy.loginfo("SkidSteer entering main loop...")
        rospy.loginfo("  Waiting for initial velocity command on /skidsteer/cmd_vel/ ...")
        while self.last_vel_cmd is None:
            self._loop_rate.sleep()
        rospy.loginfo("  Initial velocity command on /skidsteer/cmd_vel/ received!")

        # start main loop
        while not rospy.is_shutdown():

            jointstate = JointState()
            jointstate.name = self.hebi_mapping
            jointstate.position = []
            jointstate.velocity = [0.0]*WHEELS
            jointstate.effort = []

            # DiffDrive algorithm here
            # TODO: Add ramped velocity function call
            if check_if_twist_is_zero(self.last_vel_cmd, linear_threshold=0.01, angular_threshold=0.01):
                pass
            elif self.last_vel_cmd.linear.x > 0.0:
                # driving forwards
                for i, k in enumerate(self.hebi_mapping_dict):
                    if k == "fl":
                        jointstate.velocity[i] = +self.lin_vel
                    elif k == "fr":
                        jointstate.velocity[i] = -self.lin_vel
                    elif k == "bl":
                        jointstate.velocity[i] = +self.lin_vel
                    elif k == "br":
                        jointstate.velocity[i] = -self.lin_vel

            elif self.last_vel_cmd.linear.x < 0.0:
                # driving backwards
                for i, k in enumerate(self.hebi_mapping_dict):
                    if k == "fl":
                        jointstate.velocity[i] = -self.lin_vel
                    elif k == "fr":
                        jointstate.velocity[i] = +self.lin_vel
                    elif k == "bl":
                        jointstate.velocity[i] = -self.lin_vel
                    elif k == "br":
                        jointstate.velocity[i] = +self.lin_vel

            elif self.last_vel_cmd.angular.z > 0.0:
                # rotate left
                for i, k in enumerate(self.hebi_mapping_dict):
                    if k == "fl":
                        jointstate.velocity[i] = -self.ang_vel
                    elif k == "fr":
                        jointstate.velocity[i] = -self.ang_vel
                    elif k == "bl":
                        jointstate.velocity[i] = -self.ang_vel
                    elif k == "br":
                        jointstate.velocity[i] = -self.ang_vel

            elif self.last_vel_cmd.angular.z < 0.0:
                # rotate right
                for i, k in enumerate(self.hebi_mapping_dict):
                    if k == "fl":
                        jointstate.velocity[i] = +self.ang_vel
                    elif k == "fr":
                        jointstate.velocity[i] = +self.ang_vel
                    elif k == "bl":
                        jointstate.velocity[i] = +self.ang_vel
                    elif k == "br":
                        jointstate.velocity[i] = +self.ang_vel

            else:
                # undefined Twist commands
                rospy.logdebug("No logic defined for this Twist msg:")
                rospy.logdebug("%s", self.last_vel_cmd)

            self.cmd_pub.publish(jointstate)
            self._loop_rate.sleep()
        # end main loop

    def _get_joint_angles(self):
        return [self._current_jt_pos[motor] for motor in self.hebi_mapping]

    def _get_joint_velocities(self):
        return [self._current_jt_vel[motor] for motor in self.hebi_mapping]

    def _get_joint_efforts(self):
        return [self._current_jt_eff[motor] for motor in self.hebi_mapping]

    def _feedback_cb(self, msg):
        for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
            if name not in self.hebi_mapping:
                print("WARNING: arm_callback - unrecognized name!!!")
            else:
                self._current_jt_pos[name] = pos
                self._current_jt_vel[name] = vel
                self._current_jt_eff[name] = eff
                # Publish JointState() for RViz
                if not rospy.is_shutdown() and self._joint_state_pub is not None:
                    jointstate = JointState()
                    jointstate.header.stamp = rospy.Time.now()
                    jointstate.name = self.hebi_mapping
                    jointstate.position = self._get_joint_angles()
                    jointstate.velocity = [0.0] * len(jointstate.name)
                    jointstate.effort = [0.0] * len(jointstate.name)
                    self._joint_state_pub.publish(jointstate)

    def _cmd_vel_cb(self, msg):
        if isinstance(msg, Twist):
            if self.last_vel_cmd is None:
                self.last_vel_cmd = Twist()
            self.last_vel_cmd.linear.x = msg.linear.x
            self.last_vel_cmd.linear.y = msg.linear.y
            self.last_vel_cmd.linear.z = msg.linear.z
            self.last_vel_cmd.angular.x = msg.angular.x
            self.last_vel_cmd.angular.y = msg.angular.y
            self.last_vel_cmd.angular.z = msg.angular.z


if __name__ == '__main__':
    rospy.init_node('skid_steer')

    hebi_group_name = 'crab_skid_steer'
    hebi_mapping = {
                    'fl': 'FrontLeftLeg/Wheel',
                    'fr': 'FrontRightLeg/Wheel',
                    'bl': 'BackLeftLeg/Wheel',
                    'br': 'BackRightLeg/Wheel'
                   }

    crab_skid_steer = SkidSteer(hebi_group_name=hebi_group_name,
                                hebi_mapping=hebi_mapping)

    crab_skid_steer.loop()
