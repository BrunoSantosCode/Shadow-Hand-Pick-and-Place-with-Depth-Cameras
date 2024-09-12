#!/usr/bin/env python3

import tf
import rospy
import numpy as np
from copy import deepcopy
from termcolor import colored
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from moveit_commander import PlanningSceneInterface
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# UR5 velocity and acceleration
UR5_SPEED = 0.45

# Threshold for component pick
FORCE_THRESH = -100

# UR5 end-effector orientation to pick and place (acquired from get_ur5_pose.py)
UR5_ORI_X =  0.8328
UR5_ORI_Y = -0.3592
UR5_ORI_Z = -0.4107
UR5_ORI_W = -0.0934

# Conveyor height ('world')
CONV_HEIGHT = 0.795

# Default Pick Pose ('rs_camera')
pick_pose = PoseStamped()
pick_pose.header.frame_id = 'rs_camera'
pick_pose.pose.position.x = -0.155
pick_pose.pose.position.y = 0.036
pick_pose.pose.position.z = 0.539

# Factory Position
factory_pos = PoseStamped()

# Place Pose ('ra_new_base')
place_pose = PoseStamped()
place_pose.header.frame_id = 'ra_new_base'
place_pose.pose.position.x = 0.6615
place_pose.pose.position.y = 0.0260
place_pose.pose.position.z = 0.2400

# Shadow Hand Poses

# Grab Pose
grab_pose =  {'rh_FFJ1':  0.1250, 'rh_FFJ2':  1.6158, 'rh_FFJ3': -0.0586, 'rh_FFJ4': -0.0140,
              'rh_LFJ1':  0.1500, 'rh_LFJ2':  1.2320, 'rh_LFJ3':  0.0856, 'rh_LFJ4':  0.2728, 'rh_LFJ5': 0.1889, 
              'rh_MFJ1':  0.1250, 'rh_MFJ2':  1.3466, 'rh_MFJ3':  0.0315, 'rh_MFJ4': -0.1136, 
              'rh_RFJ1':  0.1750, 'rh_RFJ2':  1.3226, 'rh_RFJ3':  0.0656, 'rh_RFJ4':  0.1683, 
              'rh_THJ1': -0.3269, 'rh_THJ2':  0.4500, 'rh_THJ3':  0.0911, 'rh_THJ4':  1.0580, 'rh_THJ5': 0.7600, 
              'rh_WRJ1': -0.4788, 'rh_WRJ2': -0.0012} 

# Release Pose
release_pose = {'rh_FFJ1':  0.0000, 'rh_FFJ2': 1.0000, 'rh_FFJ3': -0.0586, 'rh_FFJ4': 0.0000,
                'rh_LFJ1':  0.0000, 'rh_LFJ2': 1.0000, 'rh_LFJ3':  0.0856, 'rh_LFJ4': 0.0000, 'rh_LFJ5': 0.1889, 
                'rh_MFJ1':  0.0000, 'rh_MFJ2': 1.0000, 'rh_MFJ3':  0.0315, 'rh_MFJ4': 0.0000, 
                'rh_RFJ1':  0.0000, 'rh_RFJ2': 1.0000, 'rh_RFJ3':  0.0656, 'rh_RFJ4': 0.0000, 
                'rh_THJ1': -0.3269, 'rh_THJ2': 0.3085, 'rh_THJ3':  0.0911, 'rh_THJ4': 1.0580, 'rh_THJ5': 0.7600, 
                'rh_WRJ1': -0.4788, 'rh_WRJ2': -0.0012} 

# Scan Pose
scan_pose = {'rh_FFJ1': 0.0168, 'rh_FFJ2': 0.8500, 'rh_FFJ3': 0.0249, 'rh_FFJ4': -0.0380, 
             'rh_LFJ1': 0.0176, 'rh_LFJ2': 0.7713, 'rh_LFJ3': 0.0944, 'rh_LFJ4': -0.2083, 'rh_LFJ5': 0.3235, 
             'rh_MFJ1': 0.0134, 'rh_MFJ2': 0.7336, 'rh_MFJ3': 0.2014, 'rh_MFJ4': -0.0040, 
             'rh_RFJ1': 0.0118, 'rh_RFJ2': 0.6101, 'rh_RFJ3': 0.4557, 'rh_RFJ4': -0.0190, 
             'rh_THJ1': 0.0926, 'rh_THJ2': 0.2704, 'rh_THJ3': -0.0846, 'rh_THJ4': 0.5808, 'rh_THJ5': 0.1703, 
             'rh_WRJ1': 0.0150, 'rh_WRJ2': -0.0136}

# Middle Finger Pose
mf_pose = {'rh_FFJ1':  1.3287, 'rh_FFJ2':  1.6853, 'rh_FFJ3':  1.5503, 'rh_FFJ4': -0.0017, 
           'rh_LFJ1':  1.3396, 'rh_LFJ2':  1.6827, 'rh_LFJ3':  1.5335, 'rh_LFJ4':  0.0064, 'rh_LFJ5': 0.0134, 
           'rh_MFJ1': -0.0058, 'rh_MFJ2':  0.0083, 'rh_MFJ3':  0.0002, 'rh_MFJ4': -0.0131, 
           'rh_RFJ1':  1.3835, 'rh_RFJ2':  1.6596, 'rh_RFJ3':  1.5531, 'rh_RFJ4':  0.0036, 
           'rh_THJ1':  0.5158, 'rh_THJ2':  0.5928, 'rh_THJ3': -0.0073, 'rh_THJ4':  1.1726, 'rh_THJ5': 0.1471, 
           'rh_WRJ1': -0.2838, 'rh_WRJ2': -0.0147}

# UR5 Scan Factory Pose
ur5_scan_fac_pose = {'ra_elbow_joint': 2.6181, 
                     'ra_shoulder_lift_joint': -3.1416, 
                     'ra_shoulder_pan_joint': -0.7854, 
                     'ra_wrist_1_joint': -2.7925, 
                     'ra_wrist_2_joint': -1.5707, 
                     'ra_wrist_3_joint': 0.0000}

# UR5 Scan Conveyor Pose
ur5_scan_pose = {'ra_elbow_joint': 0.7084, 
                 'ra_shoulder_lift_joint': -1.3309, 
                 'ra_shoulder_pan_joint': -0.0381, 
                 'ra_wrist_1_joint': -1.8570, 
                 'ra_wrist_2_joint': -2.2157, 
                 'ra_wrist_3_joint': 0.4447}

# UR5 Middle Finger Pose
ur5_mf_pose = {'ra_elbow_joint': 1.8623, 
               'ra_shoulder_lift_joint': -1.5592, 
               'ra_shoulder_pan_joint': -0.0793, 
               'ra_wrist_1_joint': -4.9804, 
               'ra_wrist_2_joint': -1.5817, 
               'ra_wrist_3_joint': -4.0222}


def fac_pos_cb(msg: PointStamped):
    global waiting_factory
    if waiting_factory:
        factory_pos.header.frame_id = msg.header.frame_id
        factory_pos.pose.position.x = msg.point.x
        factory_pos.pose.position.y = msg.point.y
        factory_pos.pose.position.z = msg.point.z
        waiting_factory = False

def obj_pos_cb(msg: PointStamped):
    global waiting_obj
    if waiting_obj:
        pick_pose.header.frame_id = msg.header.frame_id
        pick_pose.pose.position.x = msg.point.x
        pick_pose.pose.position.y = msg.point.y
        pick_pose.pose.position.z = msg.point.z
        waiting_obj = False


def set_hand_pose(pose: str, wait = True):
    """
        Sets a pre-defined pose for Shadow Hand
        @param pose - A given pose 'grab' or 'release'.
    """
    global hand_commander

    if pose == 'grab':
        hand_commander.move_to_joint_value_target_unsafe(joint_states=grab_pose, time=1.0, wait=wait, angle_degrees=False)
    elif pose == 'release':
        hand_commander.move_to_joint_value_target_unsafe(joint_states=release_pose, time=1.0, wait=wait, angle_degrees=False)
    else:
        print('\n' + colored('ERROR: "' + pose + '" hand pose is not defined!', 'red') + '\n')



def set_ur5_default_orientation():
    """
        Sets the default UR5 orientation
    """
    global arm_commander

    ur5_pose = arm_commander.get_current_pose('world')
    ur5_pose.orientation.x = UR5_ORI_X
    ur5_pose.orientation.y = UR5_ORI_Y
    ur5_pose.orientation.z = UR5_ORI_Z
    ur5_pose.orientation.w = UR5_ORI_W
    arm_commander.plan_to_pose_target(ur5_pose, 'ra_flange', alternative_method=True)
    arm_commander.execute()



def move_arm_to(obj_pose: PoseStamped, dist, mode):
    """
        Moves UR5 to the specified pose
        @param obj_pose - Object position ('world') of type PoseStamped.
        @param dist - Distance in meters to pre-position the UR5
        @param mode - Operation mode: 'pick' or 'place'
    """
    global arm_commander

    move_pose = deepcopy(obj_pose)

    try:
        # Get fingertip positions in 'world' frame
        tf_listener.waitForTransform('/world', '/rh_thtip', rospy.Time(), rospy.Duration(1.0))
        (pos_th, _) = tf_listener.lookupTransform('/world', '/rh_thtip', rospy.Time(0))
        tf_listener.waitForTransform('/world', '/rh_fftip', rospy.Time(), rospy.Duration(1.0))
        (pos_ff, _) = tf_listener.lookupTransform('/world', '/rh_fftip', rospy.Time(0))
        tf_listener.waitForTransform('/world', '/rh_lftip', rospy.Time(), rospy.Duration(1.0))
        (pos_lf, _) = tf_listener.lookupTransform('/world', '/rh_lftip', rospy.Time(0))

        # Get UR5 position in 'world' frame
        tf_listener.waitForTransform('/world', '/ra_flange', rospy.Time(), rospy.Duration(1.0))
        (pos_ur5, _) = tf_listener.lookupTransform('/world', '/ra_flange', rospy.Time(0))

        # Calculate mean fingertips position
        mean_x = (pos_th[0]+pos_ff[0]+pos_lf[0])/3
        mean_y = (pos_th[1]+pos_ff[1]+pos_lf[1])/3
        mean_z = (pos_th[2]+pos_ff[2]+pos_lf[2])/3

        # Object to UR5 vector
        vec_x = pos_ur5[0]-mean_x
        vec_y = pos_ur5[1]-mean_y
        vec_z = pos_ur5[2]-mean_z

        # Set 'ra_flange' pose from object pose
        move_pose.pose.position.x += vec_x
        move_pose.pose.position.y += vec_y
        move_pose.pose.position.z += vec_z

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("TF lookup failed.")
        exit(-1)

    # Set UR5 default orientation
    move_pose.pose.orientation.x = UR5_ORI_X
    move_pose.pose.orientation.y = UR5_ORI_Y
    move_pose.pose.orientation.z = UR5_ORI_Z
    move_pose.pose.orientation.w = UR5_ORI_W

    # Define UR5 trajectory
    trajectory = []
    if (mode == 'pick'):
        initial_pose = arm_commander.get_current_pose()
        pre_pose = deepcopy(move_pose)
        pre_pose.pose.position.z += dist
        trajectory.append(initial_pose)
        trajectory.append(pre_pose.pose)
        trajectory.append(move_pose.pose)
    elif mode == 'place':
        initial_pose = arm_commander.get_current_pose()
        back_off_pose = deepcopy(initial_pose)
        back_off_pose.position.z += 0.20
        pre_pose = deepcopy(move_pose)
        pre_pose.pose.position.z += dist
        trajectory.append(initial_pose)
        trajectory.append(back_off_pose)
        trajectory.append(pre_pose.pose)
        trajectory.append(move_pose.pose)
    else:
        print('\n' + colored('ERROR: "' + mode + '" mode is not defined!', 'red') + '\n') 
        exit(-1)

    print(colored(f'Going to pose: ({move_pose.pose.position.x}, {move_pose.pose.position.y}, {move_pose.pose.position.z})', 'cyan'))

    success = False
    while success == False:
        arm_commander.plan_to_waypoints_target(trajectory, 'world')
        success = arm_commander.execute()
        if rospy.is_shutdown(): return
        rospy.sleep(0.5)



def one_iteration(grab_pose: PoseStamped, release_pose: PoseStamped):
    """
        Executes one iteration of the clipping task
        @param clip_pose - Clip pose of type PoseStamped.
        @param clip_goal_pose - Clip goal a given pose of type PoseStamped.
    """
    global arm_commander

    #1 PICK THE COMPONENT

    set_hand_pose('release', wait=False)
    
    set_ur5_default_orientation()

    print(colored('Preparing to grab the component', 'green'))
    move_arm_to(grab_pose, 0.20, mode='pick')

    set_hand_pose('grab', wait=True)
    rospy.sleep(0.5)

    #2 CHECK IF COMPONENT WAS GRIPPED
    if hand_commander.get_joints_effort()['rh_THJ1'] < FORCE_THRESH:
        print(colored('Component grabbed', 'green'))
        if rospy.is_shutdown(): return
            
        #3 PLACE THE COMPONENT
        move_arm_to(release_pose, 0.05, mode='place')

        set_hand_pose('release', wait=True)

        print(colored('Component placed', 'green'))
        if rospy.is_shutdown(): return

    else:
        print(colored('WARNING: Component not found!', 'yellow'))
        set_hand_pose('release', wait=True)

    arm_commander.move_to_joint_value_target(ur5_scan_pose, wait=True)
    hand_commander.move_to_joint_value_target_unsafe(joint_states=scan_pose, time=1.0, wait=True)



if __name__ == "__main__":
    global arm_commander, hand_commander, waiting_obj, waiting_factory

    waiting_obj = False
    waiting_factory = False

    # Init ROS
    rospy.init_node('pick_n_place_rs')

    # Create ROS subscriber for conveyor/factory object position
    rospy.Subscriber('obj_pos', PointStamped, obj_pos_cb)
    rospy.Subscriber('factory_pos', PointStamped, fac_pos_cb)

    # Init TF listener
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()

    # Shadow Arm/Hand commander
    arm_commander = SrArmCommander(name='right_arm')
    hand_commander = SrHandCommander(name='right_hand')

    # Set control velocity and acceleration
    hand_commander.set_max_velocity_scaling_factor(1.0)
    hand_commander.set_max_acceleration_scaling_factor(1.0)
    arm_commander.set_max_velocity_scaling_factor(UR5_SPEED)
    arm_commander.set_max_acceleration_scaling_factor(UR5_SPEED)

    # Create new axis for UR5 base (the original axis is 45 degrees off)
    tf_listener.waitForTransform('/world', '/ra_base', rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = tf_listener.lookupTransform('/world', '/ra_base', rospy.Time(0))
    (roll, pitch, yaw) = euler_from_quaternion(rot)
    yaw += 135 * (3.14159 / 180)
    new_rot = quaternion_from_euler(roll, pitch, yaw)
    tf_broadcaster.sendTransform(trans, new_rot, rospy.Time.now(), '/ra_new_base', '/world')

    # Create RealSense camera frame
    tf_broadcaster.sendTransform((-1.30281583e-02, -5.91395717e-02, 2.22247086e-01),                      # translation
                                 (-1.08231509e-04, -3.89102026e-01,  9.21194496e-01, -5.50106673e-04),    # rotation
                                 rospy.Time.now(),
                                 '/rs_camera',   # child frame
                                 '/rh_forearm')  # parent frame

    # Add collision objects
    scene = PlanningSceneInterface()
    rospy.sleep(2)
    p = PoseStamped()
    p.header.frame_id = 'ra_new_base'
    p.pose.position.x = 0.00
    p.pose.position.y = 0.00
    p.pose.position.z = -0.76
    scene.add_box("ground", p, (5.0, 5.0, 0.1))
    p.pose.position.x = 1.00
    p.pose.position.y = -0.09
    p.pose.position.z = -0.27
    scene.add_box("factory_level0", p, (1.0, 0.74, 0.97))
    p.pose.position.x = 1.165
    p.pose.position.y = -0.09
    p.pose.position.z = 0.425
    scene.add_box("factory_level1", p, (0.67, 0.74, 0.42))
    p.pose.position.x = 1.00
    p.pose.position.y = 0.45
    p.pose.position.z = -0.37
    scene.add_box("conveyor", p, (1.0, 0.34, 0.77))


    print('\n' + colored('"clipping_task" ROS node is ready!', 'green') + '\n') 

    # Scan Factory Position
    arm_commander.move_to_joint_value_target(ur5_scan_fac_pose, wait=True)
    hand_commander.move_to_joint_value_target_unsafe(joint_states=scan_pose, time=1.0, wait=True)
    rospy.sleep(0.5)
    waiting_factory = True
    print('\n' + colored('Waiting for factory position...', 'yellow') + '\n') 
    while waiting_factory:
        # Update new axis for UR5 base (the original axis is 45 degrees off)
        tf_broadcaster.sendTransform(trans, new_rot, rospy.Time.now(), '/ra_new_base', '/world')
        # Update RealSense camera frame
        tf_broadcaster.sendTransform((-1.30281583e-02, -5.91395717e-02, 2.22247086e-01),                      # translation
                                     (-1.08231509e-04, -3.89102026e-01,  9.21194496e-01, -5.50106673e-04),    # rotation
                                     rospy.Time.now(),
                                     '/rs_camera',   # child frame
                                     '/rh_forearm')  # parent frame
        rospy.sleep(0.1)
    tf_listener.waitForTransform('world', factory_pos.header.frame_id, rospy.Time(), rospy.Duration(4.0))
    factory_pos = tf_listener.transformPose('world', factory_pos)
    tf_listener.waitForTransform('ra_new_base', 'world', rospy.Time(), rospy.Duration(4.0))
    factory_pos = tf_listener.transformPose('ra_new_base', factory_pos)
    place_pose.pose.position.x = factory_pos.pose.position.x + 0.0270
    place_pose.pose.position.y = factory_pos.pose.position.y + 0.0350
    place_pose.pose.position.z = factory_pos.pose.position.z + 0.1610
    print('\n' + colored('Factory position aquired!', 'green') + '\n') 

    # Convert place positions into 'world' frame
    tf_listener.waitForTransform('world', place_pose.header.frame_id, rospy.Time(), rospy.Duration(4.0))
    place_pose = tf_listener.transformPose('world', place_pose)

    # Scan Conveyor State
    arm_commander.move_to_joint_value_target(ur5_scan_pose, wait=True)
    hand_commander.move_to_joint_value_target_unsafe(joint_states=scan_pose, time=1.0, wait=True)

    waiting_obj = True

    while not rospy.is_shutdown():

        print('\n' + colored('Waiting for component...', 'yellow') + '\n') 
        
        # Check for new object to pick
        waiting_obj = True
        while(waiting_obj):
            # Update RealSense camera frame
            tf_broadcaster.sendTransform((-1.30281583e-02, -5.91395717e-02, 2.22247086e-01),                      # translation
                                         (-1.08231509e-04, -3.89102026e-01,  9.21194496e-01, -5.50106673e-04),    # rotation
                                         rospy.Time.now(),
                                         '/rs_camera',   # child frame
                                         '/rh_forearm')  # parent frame
            if rospy.is_shutdown(): exit(-1)
            rospy.sleep(0.5)

        # Pick and Place
        print('\n' + colored('Starting', 'green') + '\n')

        # Convert pick positions into 'world' frame
        pick_pose.pose.position.z += 0.05
        tf_listener.waitForTransform('world', pick_pose.header.frame_id, rospy.Time(), rospy.Duration(4.0))
        pick_pose = tf_listener.transformPose('world', pick_pose)
        pick_pose.pose.position.z = CONV_HEIGHT 

        # Start iteration
        one_iteration(pick_pose, place_pose)
        print('\n' + colored('Task completed!', 'green') + '\n')
