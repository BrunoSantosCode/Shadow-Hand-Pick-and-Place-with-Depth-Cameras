#!/usr/bin/env python3

#* * * * * * * * * * realsense2shadow_transform.py * * * * * * * * * *#
#*  version 1:                                                       *#
#*  - Converts ArUco positions from 'world' to 'rh_forearm'          *#
#*  - Acquire ArUco positions using RealSense camera in 'rs_camera'  *#
#*  - Shows ArUco positions in 'rh_forearm' and in 'rs_camera'       *#
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *#

from geometry_msgs.msg import PoseStamped
import cv2.aruco as aruco
import pyrealsense2 as rs
import numpy as np
import rospy
import math
import cv2
import tf

# Corner ArUco IDs
TOP_LEFT = 1
TOP_RIGHT = 2
BOTTOM_RIGHT = 3
BOTTOM_LEFT = 4

if __name__ == "__main__":
    # Init ROS
    rospy.init_node('realsense2shadow_transform')

    # Create listener
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Configure the pipeline to stream
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Create an align object
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Init ArUco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
    detector = aruco.ArucoDetector(dictionary, parameters)

    # ArUco 1
    aruco_1 = PoseStamped()
    aruco_1.header.frame_id = 'world'
    aruco_1.pose.position.x = 0.7443
    aruco_1.pose.position.y = -0.00347
    aruco_1.pose.position.z = 0.7815
    # ArUco 2
    aruco_2 = PoseStamped()
    aruco_2.header.frame_id = 'world'
    aruco_2.pose.position.x = 0.9441
    aruco_2.pose.position.y = -0.2093
    aruco_2.pose.position.z = 0.7823
    # ArUco 3
    aruco_3 = PoseStamped()
    aruco_3.header.frame_id = 'world'
    aruco_3.pose.position.x = 0.7875
    aruco_3.pose.position.y = -0.3584
    aruco_3.pose.position.z = 0.7801
    # ArUco 4
    aruco_4 = PoseStamped()
    aruco_4.header.frame_id = 'world'
    aruco_4.pose.position.x = 0.5853
    aruco_4.pose.position.y = -0.1584
    aruco_4.pose.position.z = 0.7782

    good_frames_counter = 0

    while not rospy.is_shutdown():

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert to OpenCV
        img = np.asanyarray(color_frame.get_data())

        # Get color intrinsics
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        # Detect ArUco
        markerCorners, markerIds, _ = detector.detectMarkers(img)
        aruco.drawDetectedMarkers(img, markerCorners)

        # Check if all corners were detected
        if (markerIds is None) or (TOP_LEFT not in markerIds) or (TOP_RIGHT not in markerIds) or (BOTTOM_LEFT not in markerIds) or (BOTTOM_RIGHT not in markerIds):
            display_image = cv2.resize(img, None, fx=0.5, fy=0.5)
            cv2.imshow('Raw Image', display_image)
            if cv2.waitKey(1) == ord('q'):
                pipeline.stop()
                cv2.destroyAllWindows()
                print("Shutting down...")
                exit()
            continue

        # Variables to save ArUco positions
        x_1 = x_2 = x_3 = x_4 = None
        y_1 = y_2 = y_3 = y_4 = None
        
        # Get ArUco positions (pixels) 
        for id in range(len(markerIds)):
            if markerIds[id] == TOP_LEFT:
                x_1 = sum(value[0] for value in markerCorners[id][0]) / 4
                y_1 = sum(value[1] for value in markerCorners[id][0]) / 4
            elif markerIds[id] == TOP_RIGHT:
                x_2 = sum(value[0] for value in markerCorners[id][0]) / 4
                y_2 = sum(value[1] for value in markerCorners[id][0]) / 4
            elif markerIds[id] == BOTTOM_RIGHT:
                x_3 = sum(value[0] for value in markerCorners[id][0]) / 4
                y_3 = sum(value[1] for value in markerCorners[id][0]) / 4
            elif markerIds[id] == BOTTOM_LEFT:
                x_4 = sum(value[0] for value in markerCorners[id][0]) / 4
                y_4 = sum(value[1] for value in markerCorners[id][0]) / 4

        # Get ArUco positions in 'rs_camera'
        depth = depth_frame.get_distance(int(x_1), int(y_1))
        aruco_1_rs = rs.rs2_deproject_pixel_to_point(color_intrinsics, [int(x_1),int(y_1)], depth)
        depth = depth_frame.get_distance(int(x_2), int(y_2))
        aruco_2_rs = rs.rs2_deproject_pixel_to_point(color_intrinsics, [int(x_2),int(y_2)], depth)
        depth = depth_frame.get_distance(int(x_3), int(y_3))
        aruco_3_rs = rs.rs2_deproject_pixel_to_point(color_intrinsics, [int(x_3),int(y_3)], depth)
        depth = depth_frame.get_distance(int(x_4), int(y_4))
        aruco_4_rs = rs.rs2_deproject_pixel_to_point(color_intrinsics, [int(x_4),int(y_4)], depth)

        # Check if ArUco are in RealSense depth range
        aruco1_ok = math.isfinite(aruco_1_rs[2]) and (aruco_1_rs[2]!=0)
        aruco2_ok = math.isfinite(aruco_2_rs[2]) and (aruco_2_rs[2]!=0)
        aruco3_ok = math.isfinite(aruco_3_rs[2]) and (aruco_3_rs[2]!=0)
        aruco4_ok = math.isfinite(aruco_4_rs[2]) and (aruco_4_rs[2]!=0)

        if aruco1_ok and aruco2_ok and aruco3_ok and aruco4_ok:

            if good_frames_counter < 50:
                good_frames_counter += 1
                print('1')
            else:
                # Display results 'rs_camera'
                print(f"ArUco 1 'rs_camera': [{aruco_1_rs[0]:.4f}, {aruco_1_rs[1]:.4f}, {aruco_1_rs[2]:.4f}]")
                print(f"ArUco 2 'rs_camera': [{aruco_2_rs[0]:.4f}, {aruco_2_rs[1]:.4f}, {aruco_2_rs[2]:.4f}]")
                print(f"ArUco 3 'rs_camera': [{aruco_3_rs[0]:.4f}, {aruco_3_rs[1]:.4f}, {aruco_3_rs[2]:.4f}]")
                print(f"ArUco 4 'rs_camera': [{aruco_4_rs[0]:.4f}, {aruco_4_rs[1]:.4f}, {aruco_4_rs[2]:.4f}]")

                # Convert ArUco positions to 'rh_forearm'
                tf_listener.waitForTransform('rh_forearm', 'world', rospy.Time(), rospy.Duration(4.0))
                aruco_1 = tf_listener.transformPose('rh_forearm', aruco_1)
                aruco_2 = tf_listener.transformPose('rh_forearm', aruco_2)
                aruco_3 = tf_listener.transformPose('rh_forearm', aruco_3)
                aruco_4 = tf_listener.transformPose('rh_forearm', aruco_4)

                # Display results 'rh_forearm'
                print()
                print("ArUco 1 'rh_forearm': [", aruco_1.pose.position.x, aruco_1.pose.position.y, aruco_1.pose.position.z, "]")
                print("ArUco 2 'rh_forearm': [", aruco_2.pose.position.x, aruco_2.pose.position.y, aruco_2.pose.position.z, "]")
                print("ArUco 3 'rh_forearm': [", aruco_3.pose.position.x, aruco_3.pose.position.y, aruco_3.pose.position.z, "]")
                print("ArUco 4 'rh_forearm': [", aruco_4.pose.position.x, aruco_4.pose.position.y, aruco_4.pose.position.z, "]")

                # Close
                pipeline.stop()
                cv2.destroyAllWindows()
                print("Done!")
                exit()
        
        else:
            print("WARNING: Object is out of range")
            display_image = cv2.resize(img, None, fx=0.5, fy=0.5)
            cv2.imshow('Raw Image', display_image)
            if cv2.waitKey(1) == ord('q'):
                pipeline.stop()
                cv2.destroyAllWindows()
                print("Shutting down...")
                exit()
            continue