import cv2
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
from numpy.linalg import svd, det

import sys

checkerboard = []

# file_num = sys.argv[1]

# bag_file_path = f"./ros-simulations/data_collection/3d/1/{file_num}.bag"
# cam_file_path = f"./ros-simulations/data_collection/3d/1/{file_num}.txt"

# with rosbag.Bag(bag_file_path, "r") as bag:
#     for topic, msg, t in bag.read_messages(topics=['/3d_lidar/depth/points']):
#         pc_data = pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)

#         points = []
#         points.extend(pc_data)

#         points = np.array(points)

#         print(np.max(points, axis=0))

#         points_y = points[:, 1]
#         points_y = np.round(points_y, 3)
#         mask = (points_y < 0.09)

#         checkerboard = points[:, :-1][mask]
#         break

checkerboard = np.load("point_cloud.npy")

print(f"Shape of the checkerboard points: {checkerboard.shape}")
avg_depth = np.mean(checkerboard[:, 2])

x_len, y_len, _ = np.max(checkerboard, axis=0) - np.min(checkerboard, axis=0)
print(f"Length of the checkerboard: {x_len:.2f} x {y_len:.2f}")

print(checkerboard[0])
print(np.max(checkerboard, axis=0), np.min(checkerboard, axis=0))


x_corner_coords, y_corner_coords, _ = np.linspace(np.min(checkerboard, axis=0), np.max(checkerboard, axis=0), num=9, axis=0).T
x_corner_coords = x_corner_coords[1:-1]
y_corner_coords = y_corner_coords[1:-1]

lidar_correspondences = np.array(np.meshgrid(x_corner_coords, y_corner_coords)).T.reshape(-1, 2)
print(lidar_correspondences)

lidar_correspondences = np.hstack((lidar_correspondences, np.full((lidar_correspondences.shape[0], 1), avg_depth)))
print(f"Shape of the lidar correspondences: {lidar_correspondences.shape}")

camera_corners = np.loadtxt("./ros-simulations/data_collection/3d/1/1.txt", delimiter=' ')
print(f"Shape of the camera corners: {camera_corners.shape}")

K = np.array([
    1345.61962890625, 0.0,              960.0, 
    0.0,              1345.61962890625, 540.0,
    0.0,              0.0,              1.0
]).reshape(3, 3)

R_t = np.array([
    [ 9.63094642e-01, -3.40295092e-02,  2.67003189e-01, -1.34093918e+01],
    [ 4.09279147e-02,  9.98955610e-01, -2.03124401e-02, -1.54841213e+01],
    [-2.66033111e-01,  3.04906860e-02,  9.63481552e-01,  1.01927801e+02]
]).reshape(3, 4)

def epnp_algorithm(world_points, image_points, K):
    """
    EPnP algorithm to solve the Perspective-n-Point problem.

    Parameters:
    - world_points: 3D coordinates of points in the world frame (n x 3)
    - image_points: 2D coordinates of corresponding points in the image frame (n x 2)
    - K: Camera intrinsic matrix

    Returns:
    - R: Rotation matrix (3 x 3)
    - t: Translation vector (3 x 1)
    """
    n = world_points.shape[0]

    # Normalize image points using the inverse of the intrinsic matrix
    inv_K = np.linalg.inv(K)

    normalized_image_points = np.dot(inv_K, np.hstack((image_points, np.ones((n, 1)))).T)
    normalized_image_points = normalized_image_points[:2, :].T

    # Compute mean of 3D points
    mean_world_point = np.mean(world_points, axis=0, keepdims=True)

    # Compute the centered 3D points
    centered_world_points = world_points - mean_world_point

    # Initialize the M matrix
    M = np.zeros((2 * n, 12))

    for i in range(n):
        X, Y, Z = centered_world_points[i]
        u, v = normalized_image_points[i]

        M[2 * i, :] = [X, Y, Z, 1, 0, 0, 0, 0, -u * X, -u * Y, -u * Z, -u]
        M[2 * i + 1, :] = [0, 0, 0, 0, X, Y, Z, 1, -v * X, -v * Y, -v * Z, -v]

    # Singular Value Decomposition of M
    _, _, Vt = svd(M)

    # Extract the solution from the last column of Vt
    P_solution = Vt[-1, :].reshape((3, 4))

    # Recover rotation and translation from the camera matrix
    R = P_solution[:, :3]
    t = P_solution[:, 3].reshape((3, 1))

    # Denormalize the translation
    t = t / np.linalg.norm(R[:, 0])

    # Ensure rotation matrix has determinant 1 (it's a proper rotation matrix)
    if np.linalg.det(R) < 0:
        R = -R
        t = -t

    return R, t

def generate_random_indices(total_points, sample_size):
    indices = np.random.choice(total_points, size=sample_size, replace=False)
    return indices

def compute_p_matrix(K, R, t):
    T = np.hstack((R, t.reshape(-1, 1)))
    P = np.dot(K, T)
    return P

def calculate_residuals(points_3D, points_2D, P):
    homogeneous_points_3D = np.vstack((points_3D.T, np.ones((1, points_3D.shape[0]))))

    projected_points = np.dot(P, homogeneous_points_3D)
    projected_points = projected_points[:2, :] / projected_points[2, :]
    residuals = np.linalg.norm(projected_points - points_2D.T, axis=0)
    return residuals

def epnp_ransac(points_3D, points_2D, K, iterations=1000, threshold=1.0):
    """EPnP algorithm with RANSAC."""
    best_P = None
    best_inliers = []
    best_residual_sum = float('inf')

    total_points = points_3D.shape[0]
    indices = np.arange(total_points)

    for _ in range(iterations):
        # Randomly sample 4 points
        sample_indices = generate_random_indices(total_points, 4)
        sample_points_3D = points_3D[sample_indices, :]
        sample_points_2D = points_2D[sample_indices, :]

        # Compute EPnP solution
        R, t = epnp_algorithm(sample_points_3D, sample_points_2D, K)

        # Compute the camera projection matrix
        P = compute_p_matrix(K, R, t)

        # Calculate residuals for all points
        residuals = calculate_residuals(points_3D, points_2D, P)

        # Count inliers (points with residuals below threshold)
        inliers = indices[residuals < threshold]

        # Update the best model if this iteration produced more inliers
        if len(inliers) > len(best_inliers):
            best_P = P
            best_inliers = inliers
            best_residual_sum = np.sum(residuals)

    return best_P, best_inliers

# best_P, best_inliers = epnp_ransac(lidar_correspondences, camera_corners, K)

# print(best_P)

dist_coeffs = np.zeros((8,))
# retval, rvec, tvec, _ = cv2.solvePnPRansac(lidar_correspondences, camera_corners, K, dist_coeffs, flags=cv2.SOLVEPNP_EPNP)
retval, rvec, tvec = cv2.solvePnP(lidar_correspondences, camera_corners, K, None)

print(rvec)

new_R, _ = cv2.Rodrigues(rvec)

print("\nEstimated Rotation:")
print(new_R)

print("\nEstimated Translation:")
print(tvec)

# print("\nCheckerboard corners:")
# print(checkerboard)