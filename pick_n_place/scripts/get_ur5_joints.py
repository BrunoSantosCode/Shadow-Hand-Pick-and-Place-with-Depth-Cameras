#!/usr/bin/env python3

import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander

if __name__ == "__main__":
    # Init ROS
    rospy.init_node('get_ur5_joints')

    # Shadow Hand commander
    arm_commander = SrArmCommander(name='right_arm')

    # Get Hand Pose
    arm_joints = arm_commander.get_joints_position()

    print("\n", arm_joints, "\n")

    print("Done!")

    exit()