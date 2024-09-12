#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from termcolor import colored
import cv2.aruco as aruco
import pyzed.sl as sl
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
MIN_RADIUS = 30
MAX_RADIUS = 70

# Object detection parameters
POSE_THRESH = 0.0075   # threshold for detecting object in same position [in meters]
TIMER_LIMIT = 0.5       # time for object to be detected as stationary [in seconds]

# Options for debug
SHOW_RAW_IMAGE = True
SHOW_MASK = False
CHECK_ARUCO = False
CONTINUOUSLY_RUN = True


# Receives PCL and pixel and retrieves the mean 3D position
def get_mean_pcl_pos(x, y, pcl):
    x = int(x)
    y = int(y)
    pcl_points = []
    good_points = 0
    for i in range(x-2, x+3):
        for j in range(y-2, y+3):
            pcl_point = pcl.get_value(int(i), int(j))[1]
            if math.isfinite(pcl_point[2]):
                pcl_points.append((pcl_point[0], pcl_point[1], pcl_point[2]))
                good_points += 1
    
    if good_points > 0:
        pcl_array = np.array(pcl_points)
        mean_x = np.sum(pcl_array[:, 0]) / good_points
        mean_y = np.sum(pcl_array[:, 1]) / good_points
        mean_z = np.sum(pcl_array[:, 2]) / good_points
        return (mean_x, mean_y, mean_z)
    else: 
        print(colored("WARNING: Object is out of range", "yellow"))
        return (-1, -1, -1)


# Receives an image and retrives the detected object coordinates
def get_object_coordinates(image, depth, pcl): 

    image = image.copy()

    # Median filter
    depth = cv2.medianBlur(depth, 25)
    # High-pass filter
    hpf = depth - cv2.GaussianBlur(depth, (15, 15), 0)
    # Threshold
    _, thresh = cv2.threshold(hpf, 150, 255, cv2.THRESH_BINARY)
    # Open
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, (15, 15))
    # Close
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (51, 51))
    # Gaussian Blur
    thresh = cv2.GaussianBlur(thresh, (7, 7), sigmaX=1.5, sigmaY=1.5)

    # DEBUG
    if SHOW_MASK:
        display_mask = cv2.resize(thresh, None, fx=0.5, fy=0.5)
        cv2.imshow('Mask', display_mask)
        cv2.waitKey(1)

    # Apply Hough transform on the blurred image. 
    detected_circles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT,
                                        dp=1.50, minDist=50,
                                        param1=50, param2=40, 
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
            # Centroids [ZED]
            obj_position = get_mean_pcl_pos(int(x), int(y), pcl)
            centroids_zed.append(obj_position)

            count += 1
    
    if count > 0:
        # Find the bottom-most object
        max_bottom_pos = 0
        max_bottom_id = -1
        for i in range(count):
            if (centroids_px[i][1] > max_bottom_pos) and (centroids_zed[i][2] != -1):
                max_bottom_pos = centroids_px[i][1]
                max_bottom_id = i
            
        # Highlight the object centroid in the image
        image = cv2.circle(image, centroids_px[max_bottom_id], 40, (255,0,0), 5) 

        return image, centroids_zed[max_bottom_id], centroids_px[max_bottom_id]
    
    else:
        return image, (-1, -1, -1), (-1, -1)
    

# Receives corner xy and check if object is close
def is_obj_close_to_corners(obj_x, obj_y, corners):
    dist_thresh = 100
    dist = np.sqrt( np.power((obj_x-corners[0][0]),2) + np.power((obj_y-corners[0][1]),2) )
    if dist < dist_thresh:
        return True
    dist = np.sqrt( np.power((obj_x-corners[1][0]),2) + np.power((obj_y-corners[1][1]),2) )
    if dist < dist_thresh:
        return True
    dist = np.sqrt( np.power((obj_x-corners[2][0]),2) + np.power((obj_y-corners[2][1]),2) )
    if dist < dist_thresh:
        return True
    dist = np.sqrt( np.power((obj_x-corners[3][0]),2) + np.power((obj_y-corners[3][1]),2) )
    if dist < dist_thresh:
        return True
    return False

# Main
if __name__ == "__main__":

    # Init ROS
    rospy.init_node('acquire_pos_zed')

    # Create ROS publisher
    pub_obj = rospy.Publisher('obj_pos', PointStamped, queue_size=1)
    pub_facory = rospy.Publisher('factory_pos', PointStamped, queue_size=1)

    # Create a Camera object
    zed = sl.Camera()

    # Create configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD2K
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # Use NEURAL depth mode
    init_params.coordinate_units = sl.UNIT.METER   # Use meter units

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

    # Pose/timer for detected object
    last_pose = (-1, -1, -1)
    start_time = time.time()

    zed_img = sl.Mat()
    zed_depth = sl.Mat()
    point_cloud = sl.Mat()

    frames = 0
    
    while not rospy.is_shutdown():

        # Grab an image
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:

            # Discard fisrt 50 frames
            if frames < 50:
                frames += 1
                continue

            # Capture left image and convert to OpenCV
            zed.retrieve_image(zed_img, sl.VIEW.LEFT)
            img = cv2.cvtColor(zed_img.get_data(), cv2.COLOR_BGRA2BGR)

            # Capture depth image and convert to OpenCV
            zed.retrieve_image(zed_depth, sl.VIEW.DEPTH)
            depth_img = cv2.cvtColor(zed_depth.get_data(), cv2.COLOR_BGRA2GRAY)

            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

            # Detect ArUco
            markerCorners, markerIds, _ = detector.detectMarkers(img)
            aruco.drawDetectedMarkers(img, markerCorners)

            # Check for factory ArUco
            if markerIds is not None:
                if FACTORY in markerIds:
                    factory_aruco_index = np.where(markerIds == FACTORY)[0][0]
                    x = sum(value[0] for value in markerCorners[factory_aruco_index][0]) / 4
                    y = sum(value[1] for value in markerCorners[factory_aruco_index][0]) / 4
                    xyz = get_mean_pcl_pos(x, y, point_cloud)
                    if xyz[2] > 0:
                        # Create point msg to publish
                        pub_point = PointStamped()
                        pub_point.header.stamp = rospy.Time.now()
                        pub_point.header.frame_id = 'zed_camera'
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
                    display_image = cv2.resize(img, None, fx=0.50, fy=0.50)
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

            # Isolate ROI
            img_mask = np.zeros_like(img).astype(img.dtype)
            depth_mask = np.zeros_like(depth_img).astype(depth_img.dtype)
            points = np.array([[x_1, y_1], [x_2, y_2], [x_3, y_3], [x_4, y_4]]).astype(int)
            cv2.fillPoly(img_mask, [points], color=(255, 255, 255))
            cv2.fillPoly(depth_mask, [points], color=255)
            img_roi = cv2.bitwise_and(img, img_mask)
            depth_roi = cv2.bitwise_and(depth_img, depth_mask)
    
            # Get detected object coordinates
            new_img, obj_centroids, obj_centroids_px = get_object_coordinates(img_roi, depth_roi, point_cloud)

            # Create point msg to publish
            pub_point = PointStamped()
            pub_point.header.stamp = rospy.Time.now()
            pub_point.header.frame_id = 'zed_camera'
            pub_point.point.x = -1
            pub_point.point.y = -1
            pub_point.point.z = -1

            # If object detected
            if (obj_centroids[0] != -1) and (not is_obj_close_to_corners(obj_centroids_px[0], obj_centroids_px[1], points)):
                # Check if coordinates are the same
                dist = np.sqrt( np.power(last_pose[0]-obj_centroids[0],2) + np.power(last_pose[1]-obj_centroids[1],2) + np.power(last_pose[2]-obj_centroids[2],2))
                print(dist)
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
                        new_img = cv2.circle(new_img, obj_centroids_px, 60, (0,255,0), 5) 
                        # Publish detected object centroid
                        pub_obj.publish(pub_point)
                else:
                    # Coordinates are not the same
                    last_pose = obj_centroids
                    start_time = time.time()

            # Display image
            display_image = cv2.resize(new_img, None, fx=0.50, fy=0.50)
            cv2.imshow('Corrected Image', display_image)   
            if SHOW_RAW_IMAGE:
                display_image = cv2.resize(img, None, fx=0.50, fy=0.50)
                cv2.imshow('Raw Image', display_image)         
            
            if not CONTINUOUSLY_RUN:
                zed.close()
                cv2.destroyAllWindows()
                print("Shutting down...")
                exit()
            
            if cv2.waitKey(1) == ord('q'):
                zed.close()
                cv2.destroyAllWindows()
                print("Shutting down...")
                exit()
                    
        else:
            print(colored("WARNING: couldn't read frame", "yellow"))