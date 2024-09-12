#!/usr/bin/env python3

#* * * * * * * * * * * zed2shadow_transform.py * * * * * * * * * * *#
#*  version 1:                                                     *#
#*  - Converts ArUco positions from 'world' to 'rh_forearm'        *#
#*  - Acquire ArUco positions using ZED camera in 'zed_camera'     *#
#*  - Shows ArUco positions in 'rh_forearm' and in 'zed_camera'    *#
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *#

from geometry_msgs.msg import PoseStamped
import cv2.aruco as aruco
import pyzed.sl as sl
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
    rospy.init_node('zed2shadow_transform')

    # Create listener
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()

    # Create a Camera object
    zed = sl.Camera()

    # Create configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD2K
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units

    # Open the camera
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
        print("Camera Open : "+repr(status)+". Exit program.")
        exit()

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()

    # Init ArUco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
    detector = aruco.ArucoDetector(dictionary, parameters)

    # ArUco 1
    aruco_1 = PoseStamped()
    aruco_1.header.frame_id = 'world'
    aruco_1.pose.position.x = 0.7690
    aruco_1.pose.position.y = 0.0049
    aruco_1.pose.position.z = 0.7826
    # ArUco 2
    aruco_2 = PoseStamped()
    aruco_2.header.frame_id = 'world'
    aruco_2.pose.position.x = 0.9788
    aruco_2.pose.position.y = -0.1906
    aruco_2.pose.position.z = 0.7840
    # ArUco 3
    aruco_3 = PoseStamped()
    aruco_3.header.frame_id = 'world'
    aruco_3.pose.position.x = 0.8274
    aruco_3.pose.position.y = -0.3521
    aruco_3.pose.position.z = 0.7817
    # ArUco 4
    aruco_4 = PoseStamped()
    aruco_4.header.frame_id = 'world'
    aruco_4.pose.position.x = 0.6174
    aruco_4.pose.position.y = -0.1569
    aruco_4.pose.position.z = 0.7793

    zed_img = sl.Mat()
    zed_pcl = sl.Mat()

    while not rospy.is_shutdown():

        # Grab an image
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:

            # Retrieve left image
            zed.retrieve_image(zed_img, sl.VIEW.LEFT)

            # Convert to OpenCV
            img = cv2.cvtColor(zed_img.get_data(), cv2.COLOR_BGRA2BGR)

            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieve_measure(zed_pcl, sl.MEASURE.XYZRGBA)

            # Detect ArUco
            markerCorners, markerIds, _ = detector.detectMarkers(img)
            aruco.drawDetectedMarkers(img, markerCorners)

            # Check if all corners were detected
            if (markerIds is None) or (TOP_LEFT not in markerIds) or (TOP_RIGHT not in markerIds) or (BOTTOM_LEFT not in markerIds) or (BOTTOM_RIGHT not in markerIds):
                display_image = cv2.resize(img, None, fx=0.5, fy=0.5)
                cv2.imshow('Raw Image', display_image)
                if cv2.waitKey(1) == ord('q'):
                    zed.close()
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

            # Get ArUco positions in 'zed_camera'
            aruco_1_zed = zed_pcl.get_value(int(x_1), int(y_1))[1]
            aruco_2_zed = zed_pcl.get_value(int(x_2), int(y_2))[1]
            aruco_3_zed = zed_pcl.get_value(int(x_3), int(y_3))[1]
            aruco_4_zed = zed_pcl.get_value(int(x_4), int(y_4))[1]

            # Check if ArUco are in ZED depth range
            if math.isfinite(aruco_1_zed[2]) and math.isfinite(aruco_2_zed[2]) and math.isfinite(aruco_3_zed[2]) and math.isfinite(aruco_4_zed[2]):
                # Display results 'zed_camera'
                print("ArUco 1 'zed_camera':", aruco_1_zed[:3])
                print("ArUco 2 'zed_camera':", aruco_2_zed[:3])
                print("ArUco 3 'zed_camera':", aruco_3_zed[:3])
                print("ArUco 4 'zed_camera':", aruco_4_zed[:3])

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
                zed.close()
                cv2.destroyAllWindows()
                print("Done!")
                exit()
            
            else:
                print("WARNING: Object is out of range")
                display_image = cv2.resize(img, None, fx=0.5, fy=0.5)
                cv2.imshow('Raw Image', display_image)
                if cv2.waitKey(1) == ord('q'):
                    zed.close()
                    cv2.destroyAllWindows()
                    print("Shutting down...")
                    exit()
                continue