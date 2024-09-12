#!/usr/bin/env python3

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander


# Shadow Hand Poses

# Grab Pose
grab_pose =  {'rh_FFJ1':  0.1250, 'rh_FFJ2':  1.6158, 'rh_FFJ3': -0.0586, 'rh_FFJ4': -0.0140,
              'rh_LFJ1':  0.1500, 'rh_LFJ2':  1.2320, 'rh_LFJ3':  0.0856, 'rh_LFJ4':  0.2728, 'rh_LFJ5': 0.1889, 
              'rh_MFJ1':  0.1250, 'rh_MFJ2':  1.3466, 'rh_MFJ3':  0.0315, 'rh_MFJ4': -0.1136, 
              'rh_RFJ1':  0.1750, 'rh_RFJ2':  1.3226, 'rh_RFJ3':  0.0656, 'rh_RFJ4':  0.1683, 
              'rh_THJ1': -0.3269, 'rh_THJ2':  0.4000, 'rh_THJ3':  0.0911, 'rh_THJ4':  1.0580, 'rh_THJ5': 0.7600, 
              'rh_WRJ1': -0.4788, 'rh_WRJ2': -0.0012} 

# Release Pose
release_pose = {'rh_FFJ1':  0.0000, 'rh_FFJ2': 1.0000, 'rh_FFJ3': -0.0586, 'rh_FFJ4': 0.0000,
                'rh_LFJ1':  0.0000, 'rh_LFJ2': 1.0000, 'rh_LFJ3':  0.0856, 'rh_LFJ4': 0.0000, 'rh_LFJ5': 0.1889, 
                'rh_MFJ1':  0.0000, 'rh_MFJ2': 1.0000, 'rh_MFJ3':  0.0315, 'rh_MFJ4': 0.0000, 
                'rh_RFJ1':  0.0000, 'rh_RFJ2': 1.0000, 'rh_RFJ3':  0.0656, 'rh_RFJ4': 0.0000, 
                'rh_THJ1': -0.3269, 'rh_THJ2': 0.3085, 'rh_THJ3':  0.0911, 'rh_THJ4': 1.0580, 'rh_THJ5': 0.7600, 
                'rh_WRJ1': -0.4788, 'rh_WRJ2': -0.0012} 

# Scan Pose
scan_pose = {'rh_FFJ1': 1.5707, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.00,
             'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.00,
             'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.00,
             'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.00, 'rh_LFJ5': 0.0000,
             'rh_THJ1': 0.8975, 'rh_THJ2': 0.6885, 'rh_THJ3': 0.0000, 'rh_THJ4': 0.00, 'rh_THJ5': -1.0564,
             'rh_WRJ1': 0.4000, 'rh_WRJ2': 0.0000}

if __name__ == "__main__":
    # Init ROS
    rospy.init_node('set_shadow_joints')

    # Shadow Hand commander
    hand_commander = SrHandCommander(name='right_hand')

    # Set control velocity and acceleration
    speed = 1.0
    hand_commander.set_max_velocity_scaling_factor(speed)
    hand_commander.set_max_acceleration_scaling_factor(speed)

    # Set Hand Pose
    while(not rospy.is_shutdown()):
        print('Release')
        hand_commander.move_to_joint_value_target_unsafe(joint_states=release_pose, time=1.0, wait=True, angle_degrees=False)
        rospy.sleep(1.0)
        print('Grab')
        hand_commander.move_to_joint_value_target_unsafe(joint_states=grab_pose, time=1.0, wait=True, angle_degrees=False)
        rospy.sleep(1.0)
        print('Scan')
        hand_commander.move_to_joint_value_target_unsafe(joint_states=scan_pose, time=1.0, wait=True, angle_degrees=False)
        rospy.sleep(1.0)

    print("Done!")

    exit()