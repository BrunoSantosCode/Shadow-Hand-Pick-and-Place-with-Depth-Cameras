import numpy as np
from scipy.spatial.transform import Rotation

def transform_point(point, homogeneous_matrix):
    # Append 1 to the coordinates to make it homogeneous
    point_homogeneous = np.hstack((point, 1))
    
    # Convert back to Cartesian coordinates
    transformed_point = np.dot(homogeneous_matrix, point_homogeneous)
    
    return transformed_point[:3]

# ArUco points in camera frame
aruco_1_cam = np.array([-0.08923007, 0.12787527, 0.50802279])
aruco_2_cam = np.array([-0.08876848, -0.13644876, 0.61567229])
aruco_3_cam = np.array([0.13122678, -0.13194713, 0.62563699])
aruco_4_cam = np.array([0.13140811, 0.13240215, 0.51719236])

# ArUco points in 'rh_forearm' frame
aruco_1_rh = np.array([0.14855780209467706, -0.026038453816030982, 0.6411834842114928])
aruco_2_rh = np.array([0.14919050099172415, 0.24247794343945106, 0.7418651314816971])
aruco_3_rh = np.array([-0.07199832313563159, 0.23906619381733196, 0.7504304128941012])
aruco_4_rh = np.array([-0.07295527756481412, -0.029754526965320327, 0.650714294256662])

# Constructing matrices A and B
A = np.vstack((aruco_1_cam, aruco_2_cam, aruco_3_cam, aruco_4_cam)).T
B = np.vstack((aruco_1_rh, aruco_2_rh, aruco_3_rh, aruco_4_rh)).T

# Compute centroids of points
centroid_A = np.mean(A, axis=1)
centroid_B = np.mean(B, axis=1)

# Center the points
A_centered = A - centroid_A[:, np.newaxis]
B_centered = B - centroid_B[:, np.newaxis]

# Compute H matrix
H = B_centered @ A_centered.T

# Perform Singular Value Decomposition
U, S, Vt = np.linalg.svd(H)

# Compute rotation matrix
R = Vt.T @ U.T

# Handle special cases where det(R) is -1
if np.linalg.det(R) < 0:
    Vt[-1,:] *= -1
    R = Vt.T @ U.T

# Compute translation vector
t = -R @ centroid_A + centroid_B

# Homogeneous transformation matrix
homogeneous_matrix = np.eye(4)
homogeneous_matrix[:3, :3] = R
homogeneous_matrix[:3, 3] = t

print("Homogeneous Transformation Matrix:")
print(homogeneous_matrix)

quaternion = Rotation.from_matrix(R).as_quat()

print("Quaternion representation:")
print(quaternion)

aruco_1_rh_trans = transform_point(aruco_1_cam, homogeneous_matrix)
print("Original ArUco 1 'rh_forearm':", aruco_1_rh)
print("Transformed ArUco 1 'rh_forearm':", aruco_1_rh_trans)
aruco_2_rh_trans = transform_point(aruco_2_cam, homogeneous_matrix)
print("Original ArUco 2 'rh_forearm':", aruco_2_rh)
print("Transformed ArUco 2 'rh_forearm':", aruco_2_rh_trans)
aruco_3_rh_trans = transform_point(aruco_3_cam, homogeneous_matrix)
print("Original ArUco 3 'rh_forearm':", aruco_3_rh)
print("Transformed ArUco 3 'rh_forearm':", aruco_3_rh_trans)
aruco_4_rh_trans = transform_point(aruco_4_cam, homogeneous_matrix)
print("Original ArUco 4 'rh_forearm':", aruco_4_rh)
print("Transformed ArUco 4 'rh_forearm':", aruco_4_rh_trans)