#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    # Init ROS
    rospy.init_node('get_ur5_joints')

    # Create listener
    listener = tf.TransformListener()

    try:
        # Get the transformation from the 'world' frame to 'ra_flange' frame
        listener.waitForTransform('/world', '/ra_flange', rospy.Time(), rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform('/world', '/ra_flange', rospy.Time(0))

        # Print the position and orientation
        print("UR5 flange position: ", trans)
        print("UR5 flange orientation: ", rot)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("TF lookup failed.")

    print("Done!")

    exit()