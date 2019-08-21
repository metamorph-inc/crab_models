#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from hebiros.srv import AddGroupFromNamesSrv, SendCommandWithAcknowledgementSrv
from hebiros.msg import TrajectoryAction, TrajectoryGoal, WaypointMsg, CommandMsg
from actionlib.simple_action_client import SimpleActionClient
from hebiros.msg import CommandMsg


hebi_mapping = None

trajectory_action_client = None

current_jt_pos = {}
current_jt_vel = {}
current_jt_eff = {}

arm_cmd = None
arm_cmd_cnt = 0

jointstate = None
hold = False
hold_position = []


def feedback_cb(msg):
    for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
        current_jt_pos[name] = pos
        current_jt_vel[name] = vel
        current_jt_eff[name] = eff

    if hold:
        cmd_pub.publish(hold_position)


def arm_cmds_cb(msg):
    global arm_cmd, arm_cmd_cnt
    arm_cmd = msg.data
    arm_cmd_cnt += 1
    if arm_cmd_cnt > 8:
        arm_cmd_cnt = 1


def get_jt_angles():
    return [current_jt_pos[x] for x in hebi_mapping_flat]


def get_traj_goal_to_target_jt_pos(target_jt_pos, time):
    goal = TrajectoryGoal()
    i_wp = WaypointMsg()
    i_wp.names = hebi_mapping_flat
    i_wp.positions = get_jt_angles()
    i_wp.velocities = [0.0]*len(hebi_mapping_flat)
    i_wp.accelerations = [0.0]*len(hebi_mapping_flat)
    f_wp = WaypointMsg()
    f_wp.positions = target_jt_pos
    f_wp.velocities = [0.0]*len(hebi_mapping_flat)
    f_wp.accelerations = [0.0]*len(hebi_mapping_flat)
    goal.waypoints = [i_wp, f_wp]
    goal.times = [0, time]
    return goal


def execute_trajectory_to_target_jt_pos(target_jt_pos, time):
    goal = get_traj_goal_to_target_jt_pos(target_jt_pos, time)
    trajectory_action_client.send_goal(goal)
    trajectory_action_client.wait_for_result()


if __name__ == '__main__':
    rospy.init_node('hold_up_arms')

    hebi_group_name = 'crab_hold_up_arms'
    hebi_mapping = [['LeftArm/Base', 'LeftArm/Shoulder', 'LeftArm/Elbow'],
                    ['RightArm/Base', 'RightArm/Shoulder', 'RightArm/Elbow']]
    hebi_mapping_flat = [motor for arm in hebi_mapping for motor in arm]

    hebi_families = []
    hebi_names = []
    for motor in hebi_mapping_flat:
        family, name = motor.split('/')
        hebi_families.append(family)
        hebi_names.append(name)
    rospy.loginfo("  hebi_group_name: %s", hebi_group_name)
    rospy.loginfo("  hebi_families: %s", hebi_families)
    rospy.loginfo("  hebi_names: %s", hebi_names)

    # Create a service client to create a group
    set_hebi_group = rospy.ServiceProxy('/hebiros/add_group_from_names', AddGroupFromNamesSrv)
    # Topic to receive feedback from a group
    hebi_group_feedback_topic = "/hebiros/" + hebi_group_name + "/feedback/joint_state"
    rospy.loginfo("  hebi_group_feedback_topic: %s", "/hebiros/" + hebi_group_name + "/feedback/joint_state")
    # Topic to send commands to a group
    hebi_group_command_topic = "/hebiros/" + hebi_group_name + "/command/joint_state"
    rospy.loginfo("  hebi_group_command_topic: %s", "/hebiros/" + hebi_group_name + "/command/joint_state")
    # Call the /hebiros/add_group_from_names service to create a group
    rospy.loginfo("  Waiting for AddGroupFromNamesSrv at %s ...", '/hebiros/add_group_from_names')
    rospy.wait_for_service('/hebiros/add_group_from_names')  # block until service server starts
    rospy.loginfo("  AddGroupFromNamesSrv AVAILABLE.")
    set_hebi_group(hebi_group_name, hebi_names, hebi_families)

    trajectory_action_client = SimpleActionClient("/hebiros/"+hebi_group_name+"/trajectory", TrajectoryAction)
    rospy.loginfo("  Waiting for TrajectoryActionServer at %s ...", "/hebiros/"+hebi_group_name+"/trajectory")
    trajectory_action_client.wait_for_server()  # block until action server starts
    rospy.loginfo("  TrajectoryActionServer AVAILABLE.")

    # Create a service client to set group settings
    change_group_settings = rospy.ServiceProxy("/hebiros/"+hebi_group_name+"/send_command_with_acknowledgement",
                                               SendCommandWithAcknowledgementSrv)
    rospy.loginfo("  Waiting for SendCommandWithAcknowledgementSrv at %s ...", "/hebiros/"+hebi_group_name+"/send_command_with_acknowledgement")
    rospy.wait_for_service("/hebiros/"+hebi_group_name+"/send_command_with_acknowledgement")  # block until service server starts
    cmd_msg = CommandMsg()
    cmd_msg.name = hebi_mapping_flat
    cmd_msg.settings.name = cmd_msg.name
    cmd_msg.settings.velocity_gains.name = cmd_msg.name
    cmd_msg.settings.position_gains.kp = [0.5, 1.5, 1.5, 0.5, 1.5, 1.5]
    cmd_msg.settings.position_gains.ki = [0]*6
    cmd_msg.settings.position_gains.kd = [0.1]*6
    cmd_msg.settings.position_gains.i_clamp = [0]*6
    change_group_settings(cmd_msg)

    # Feedback/Command
    fbk_sub = rospy.Subscriber(hebi_group_feedback_topic, JointState, feedback_cb)
    cmd_pub = rospy.Publisher(hebi_group_command_topic, JointState, queue_size=1)

    from_master = rospy.Subscriber("/crab/arm_cmds", String, arm_cmds_cb)

    rate = rospy.Rate(100)

    jointstate = JointState()
    jointstate.name = hebi_mapping_flat
    jointstate.velocity = []
    jointstate.effort = []

    while not rospy.is_shutdown():
        if arm_cmd_cnt == 1:
            rospy.loginfo("Raise arms.")
            jointstate.position = [0.0, -0.7, 0.7, 0.0, 0.7, -0.7]
            if arm_cmd is not None:
                hold = False
                execute_trajectory_to_target_jt_pos(jointstate.position, 3)
                hold_position = copy.deepcopy(jointstate)
                hold = True
                arm_cmd = None

        elif arm_cmd_cnt == 2:
            rospy.loginfo("Hold arms forward.")
            jointstate.position = [-1.571, -0.7, 0.7, 1.571, 0.7, -0.7]
            if arm_cmd is not None:
                hold = False
                execute_trajectory_to_target_jt_pos(jointstate.position, 2)
                hold_position = copy.deepcopy(jointstate)
                hold = True
                arm_cmd = None

        elif arm_cmd_cnt == 3:
            rospy.loginfo("Lower arms.")
            jointstate.position = [-1.571, 0.0, -0.7, 1.571, 0.0, 0.7]
            if arm_cmd is not None:
                hold = False
                execute_trajectory_to_target_jt_pos(jointstate.position, 4)
                hold_position = copy.deepcopy(jointstate)
                hold = True
                arm_cmd = None

        elif arm_cmd_cnt == 4:
            rospy.loginfo("Lower elbow joints.")
            jointstate.position = [-1.571, 0.0, 0.6, 1.571, 0.0, -0.6]
            if arm_cmd is not None:
                hold = False
                execute_trajectory_to_target_jt_pos(jointstate.position, 2)
                hold_position = copy.deepcopy(jointstate)
                hold = True
                arm_cmd = None

        elif arm_cmd_cnt == 5:
            rospy.loginfo("Grab.")
            jointstate.position = [-2.2, 0.0, 0.6, 2.2, 0.0, -0.6]
            if arm_cmd is not None:
                hold = False
                execute_trajectory_to_target_jt_pos(jointstate.position, 2)
                hold_position = copy.deepcopy(jointstate)
                hold = True
                arm_cmd = None

        elif arm_cmd_cnt == 6:
            rospy.loginfo("Lift.")
            jointstate.position = [-2.2, -0.7, 0.6, 2.2, 0.7, -0.6]
            if arm_cmd is not None:
                hold = False
                execute_trajectory_to_target_jt_pos(jointstate.position, 2)
                hold_position = copy.deepcopy(jointstate)
                hold = True
                arm_cmd = None

        elif arm_cmd_cnt == 7:
            rospy.loginfo("Drop.")
            jointstate.position = [-1.6, -0.7, 0.6, 1.6, 0.7, -0.6]
            if arm_cmd is not None:
                hold = False
                execute_trajectory_to_target_jt_pos(jointstate.position, 2)
                hold_position = copy.deepcopy(jointstate)
                hold = True
                arm_cmd = None

        elif arm_cmd_cnt == 8:
            rospy.loginfo("Lower arms.")
            jointstate.position = [-1.6, -0.5, 0.6, 1.6, 0.5, -0.6]
            if arm_cmd is not None:
                hold = False
                execute_trajectory_to_target_jt_pos(jointstate.position, 2)
                hold_position = copy.deepcopy(jointstate)
                hold = True
                arm_cmd = None

        rate.sleep()
