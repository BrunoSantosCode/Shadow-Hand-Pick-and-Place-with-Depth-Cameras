#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

if __name__ == "__main__":
    # Init ROS
    rospy.init_node('get_shadow_mftip')

    # Create listener
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()

    # Create new axis for UR5 base (the original axis is 45 degrees off)
    tf_listener.waitForTransform('/world', '/ra_base', rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = tf_listener.lookupTransform('/world', '/ra_base', rospy.Time(0))
    (roll, pitch, yaw) = euler_from_quaternion(rot)
    yaw += 135 * (3.14159 / 180)
    new_rot = quaternion_from_euler(roll, pitch, yaw)
    tf_broadcaster.sendTransform(trans, new_rot, rospy.Time.now(), '/ra_new_base', '/world')

    try:
        # Get the transformation from the 'world' frame to 'rh_mftip' frame
        tf_listener.waitForTransform('/world', '/rh_mftip', rospy.Time(), rospy.Duration(1.0))
        (trans, rot) = tf_listener.lookupTransform('/world', '/rh_mftip', rospy.Time(0))

        # Print the position and orientation
        print("Shadow Hand MFTIP position: ", trans)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("TF lookup failed.")

    print("Done!")

    exit()