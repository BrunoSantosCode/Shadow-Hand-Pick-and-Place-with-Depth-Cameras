#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from termcolor import colored
import cv2.aruco as aruco
import pyrealsense2 as rs
import numpy as np
import math
import time
import cv2

# Corner ArUco IDs
TOP_LEFT = 1
TOP_RIGHT = 2
BOTTOM_LEFT = 4
BOTTOM_RIGHT = 3
FACTORY = 10

# Pieces radius thresh
MIN_RADIUS = 10
MAX_RADIUS = 50

# Object detection parameters
POSE_THRESH = 0.006     # threshold for detecting object in same position [in meters]
TIMER_LIMIT = 0.5       # time for object to be detected as stationary [in seconds]

# Options for debug
SHOW_RAW_IMAGE = True
CHECK_ARUCO = False
CONTINUOUSLY_RUN = True


# Receives PCL and pixel and retrieves the mean 3D position
def get_mean_pcl_pos(intrinsics, x, y):
    x = int(x)
    y = int(y)
    pcl_points = []
    good_points = 0
    for i in range(x-1, x+2):
        if i >= 1080:
            continue
        for j in range(y-1, y+2):
            if j >= 720 :
                continue
            depth = depth_frame.get_distance(x, y)
            pcl_point = rs.rs2_deproject_pixel_to_point(intrinsics, [x,y], depth)
            if math.isfinite(pcl_point[2]):
                pcl_points.append((pcl_point[0], pcl_point[1], pcl_point[2]))
                good_points += 1
    
    if good_points > 0:
        pcl_array = np.array(pcl_points)
        mean_x = np.sum(pcl_array[:, 0]) / good_points
        mean_y = np.sum(pcl_array[:, 1]) / good_points
        mean_z = np.sum(pcl_array[:, 2]) / good_points
        mean_z = depth_frame.get_distance(x, y)
        return (mean_x, mean_y, mean_z)
    else: 
        print(colored("WARNING: Object is out of range", "yellow"))
        return (-1, -1, -1)


# Receives an image and retrives the detected object coordinates
def get_object_coordinates(image, intrinsics): 

    image = image.copy()

    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Hough transform on the blurred image.    
    detected_circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT,
                                        dp=1.0, minDist=35,
                                        param1=75, param2=25, 
                                        minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS) 

    count = 0
    centroids_px = []
    centroids_zed = []

    # Loop through each component 
    if detected_circles is not None: 
        # Convert the circle parameters a, b and r to integers. 
        detected_circles = np.uint16(np.around(detected_circles)) 
        for pt in detected_circles[0, :]: 
            # Centroids [pixels]
            x, y = pt[0], pt[1]
            centroids_px.append((int(x), int(y)))
            # Centroids [RealSense]
            obj_position = get_mean_pcl_pos(intrinsics, int(x), int(y))
            centroids_zed.append(obj_position)

            count += 1
    
    if count > 0:
        # Find the bottom-most object
        max_bottom_pos = 0
        max_bottom_id = -1
        for i in range(count):
            # Highlight the object centroid in the image
            image = cv2.circle(image, centroids_px[i], 25, (255,0,0), 5) 
            if (centroids_px[i][1] > max_bottom_pos) and (centroids_zed[i][2] != -1):
                max_bottom_pos = centroids_px[i][1]
                max_bottom_id = i

        return image, centroids_zed[max_bottom_id], centroids_px[max_bottom_id]
    
    else:
        return image, (-1, -1, -1), (-1, -1)
    

# Main
if __name__ == "__main__":

    # Init ROS
    rospy.init_node('acquire_pos_rs')

    # Create ROS publisher
    pub_obj = rospy.Publisher('obj_pos', PointStamped, queue_size=1)
    pub_facory = rospy.Publisher('factory_pos', PointStamped, queue_size=1)

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

    # Pose/timer for detected object
    last_pose = (-1, -1, -1)
    start_time = time.time()

    # Delay to start RealSense camera
    rospy.sleep(1.0)

    while not rospy.is_shutdown():

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert to OpenCV
        color_image = np.asanyarray(color_frame.get_data())

        # Get color intrinsics
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        # Detect ArUco
        markerCorners, markerIds, _ = detector.detectMarkers(color_image)
        aruco.drawDetectedMarkers(color_image, markerCorners)

        # Check for factory ArUco
        if markerIds is not None:
            if FACTORY in markerIds:
                factory_aruco_index = np.where(markerIds == FACTORY)[0][0]
                x = sum(value[0] for value in markerCorners[factory_aruco_index][0]) / 4
                y = sum(value[1] for value in markerCorners[factory_aruco_index][0]) / 4
                xyz = get_mean_pcl_pos(color_intrinsics, x, y)
                if xyz[2] > 0:
                    # Create point msg to publish
                    pub_point = PointStamped()
                    pub_point.header.stamp = rospy.Time.now()
                    pub_point.header.frame_id = 'rs_camera'
                    pub_point.point.x = xyz[0]
                    pub_point.point.y = xyz[1]
                    pub_point.point.z = xyz[2]
                    pub_facory.publish(pub_point)

        # Check if all ArUco detected
        if CHECK_ARUCO:
            if markerIds is not None:
                if TOP_LEFT in markerIds:
                    print(colored("TOP LEFT ArUco: detected", "green"))
                else:
                    print(colored("TOP LEFT ArUco: not detected", "red"))
                if TOP_RIGHT in markerIds:
                    print(colored("TOP RIGHT ArUco: detected", "green"))
                else:
                    print(colored("TOP RIGHT ArUco: not detected", "red"))
                if BOTTOM_LEFT in markerIds:
                    print(colored("BOTTOM LEFT ArUco: detected", "green"))
                else:
                    print(colored("BOTTOM LEFT ArUco: not detected", "red"))
                if BOTTOM_RIGHT in markerIds:
                    print(colored("BOTTOM RIGHT ArUco: detected\n", "green"))
                else:
                    print(colored("BOTTOM RIGHT ArUco: not detected\n", "red"))
            else:
                print(colored("TOP LEFT ArUco: not detected", "red"))
                print(colored("TOP RIGHT ArUco: not detected", "red"))
                print(colored("BOTTOM LEFT ArUco: not detected", "red"))
                print(colored("BOTTOM RIGHT ArUco: not detected\n", "red"))

        # Check if all corners were detected
        if (markerIds is None) or (TOP_LEFT not in markerIds) or (TOP_RIGHT not in markerIds) or (BOTTOM_LEFT not in markerIds) or (BOTTOM_RIGHT not in markerIds):
            if SHOW_RAW_IMAGE:
                display_image = cv2.resize(color_image, None, fx=0.75, fy=0.75)
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


        # Isolate ROI
        img_mask = np.zeros_like(color_image).astype(color_image.dtype)
        points = np.array([[x_1, y_1], [x_2, y_2], [x_3, y_3], [x_4, y_4]]).astype(int)
        cv2.fillPoly(img_mask, [points], color=(255, 255, 255))
        img_roi = cv2.bitwise_and(color_image, img_mask)

        # Get detected object coordinates
        new_img, obj_centroids, obj_centroids_px = get_object_coordinates(img_roi, color_intrinsics)

        # Create point msg to publish
        pub_point = PointStamped()
        pub_point.header.stamp = rospy.Time.now()
        pub_point.header.frame_id = 'rs_camera'
        pub_point.point.x = -1
        pub_point.point.y = -1
        pub_point.point.z = -1

        # If object detected
        if (obj_centroids[0] != -1):
            # Check if coordinates are the same
            dist = np.sqrt( np.power(last_pose[0]-obj_centroids[0],2) + np.power(last_pose[1]-obj_centroids[1],2) + np.power(last_pose[2]-obj_centroids[2],2))
            if dist < POSE_THRESH:
                # Coordinates are the same
                elapsed_time = time.time() - start_time
                if elapsed_time > TIMER_LIMIT:
                    pub_point.point.x = obj_centroids[0]
                    pub_point.point.y = obj_centroids[1]
                    pub_point.point.z = obj_centroids[2]
                    print("Object coordinates: ({:.3f}, {:.3f}, {:.3f})".format(obj_centroids[0], obj_centroids[1], obj_centroids[2]))
                    print("Stationary time: {:.1f} seconds".format(elapsed_time))
                    # Highlight the object centroid in the image
                    new_img = cv2.circle(new_img, obj_centroids_px, 30, (0,255,0), 5) 
                    # Publish detected object centroid
                    pub_obj.publish(pub_point)
            else: 
                # Coordinates are not the same
                last_pose = obj_centroids
                start_time = time.time()

        # Display image
        display_image = cv2.resize(new_img, None, fx=0.75, fy=0.75)
        cv2.imshow('Corrected Image', display_image)   
        if SHOW_RAW_IMAGE:
            display_image = cv2.resize(color_image, None, fx=0.75, fy=0.75)
            cv2.imshow('Raw Image', display_image)         
        
        if not CONTINUOUSLY_RUN:
            pipeline.stop()
            cv2.destroyAllWindows()
            print("Shutting down...")
            exit()
        
        if cv2.waitKey(1) == ord('q'):
            pipeline.stop()
            cv2.destroyAllWindows()
            print("Shutting down...")
            exit()
    