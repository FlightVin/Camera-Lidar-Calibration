from __future__ import print_function

# Built-in modules
import os
import sys
import time
import threading
import multiprocessing

# External modules
import cv2
import numpy as np
import pandas as pd
import matplotlib.cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ROS modules
PKG = 'lidar_camera_calibration'
import roslib; roslib.load_manifest(PKG)
import rosbag
import rospy
import tf2_ros
import ros_numpy
import image_geometry
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_matrix
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from pyntcloud import PyntCloud
from sensor_msgs import point_cloud2

# Global variables
OUSTER_LIDAR = False
PAUSE = False
FIRST_TIME = True
KEY_LOCK = threading.Lock()
TF_BUFFER = None
TF_LISTENER = None
CV_BRIDGE = CvBridge()
CAMERA_MODEL = image_geometry.PinholeCameraModel()

# Global paths
PKG_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
CALIB_PATH = '3D-lidar-camera-calib/src/lidar_camera_calibration/calibration_data/lidar_camera_calibration'

def save_data(data, filename, folder, is_image=False):
    # Empty data
    if not len(data): return

    # Handle filename
    filename = os.path.join(PKG_PATH, os.path.join(folder, filename))
    
    # Create folder
    try:
        os.makedirs(os.path.join(PKG_PATH, folder))
    except OSError:
        if not os.path.isdir(os.path.join(PKG_PATH, folder)): raise

    # Save image
    if is_image:
        cv2.imwrite(filename, data)
        return

    # Save points data
    if os.path.isfile(filename):
        rospy.logwarn('Updating file: %s' % filename)
        data = np.vstack((np.load(filename), data))
    np.save(filename, data)

def extract_points_2D(img_msg, now, rectify=False):
    # Log PID
    rospy.loginfo('2D Picker PID: [%d]' % os.getpid())

    # Read image using CV bridge
    try:
        img = CV_BRIDGE.imgmsg_to_cv2(img_msg, 'bgr8')
    except CvBridgeError as e: 
        rospy.logerr(e)
        return

    # Rectify image
    if rectify: CAMERA_MODEL.rectifyImage(img, img)
    disp = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2RGB)

    # Setup matplotlib GUI
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title('Select 2D Image Points - %d' % now)
    ax.set_axis_off()
    ax.imshow(disp)

    # Pick points
    corners = []
    def onclick(event):
        x = event.xdata
        y = event.ydata
        if (x is None) or (y is None): return

        # Display the picked point
        corners.append((x, y))

        if len(corners) > 1:
            # Draw the line
            ax.plot([corners[-2][0], corners[-1][0]], [corners[-2][1], corners[-1][1]], 'r')
            ax.figure.canvas.draw_idle()

    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()

    exit(0)

    # Save corner points and image
    rect = '_rect' if rectify else ''
    if len(corners) > 1: del corners[-1] # Remove last duplicate
    save_data(corners, 'img_corners%s.npy' % (rect), CALIB_PATH)
    save_data(img, 'image_color%s-%d.jpg' % (rect, now), 
        os.path.join(CALIB_PATH, 'images'), True)

def extract_points_3D(velodyne, now):
    # Log PID
    rospy.loginfo('3D Picker PID: [%d]' % os.getpid())

    # Extract points data
    points = point_cloud2.read_points(velodyne, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(points))

    # Select points within chessboard range
    inrange = np.where((points[:, 1] < 0.146))

    points = points[inrange[0]]

    if points.shape[0] > 5:
        rospy.loginfo('PCL points available: %d', points.shape[0])
    else:
        rospy.logwarn('Very few PCL points available in range')
        return

    # Color map for the points
    cmap = matplotlib.cm.get_cmap('hsv')
    colors = cmap(points[:, -1] / np.max(points[:, -1]))

    # Setup matplotlib GUI
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('Select 3D LiDAR Points - %d' % now, color='white')
    ax.set_axis_off()
    ax.set_facecolor((0, 0, 0))
    scatter = ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors, s=2, picker=5)

    # Equalize display aspect ratio for all axes
    max_range = (np.array([points[:, 0].max() - points[:, 0].min(), 
        points[:, 1].max() - points[:, 1].min(),
        points[:, 2].max() - points[:, 2].min()]).max() / 2.0)
    mid_x = (points[:, 0].max() + points[:, 0].min()) * 0.5
    mid_y = (points[:, 1].max() + points[:, 1].min()) * 0.5
    mid_z = (points[:, 2].max() + points[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    picked_points = []

    def onpick(event):
        ind = event.ind[0]
        x, y, z = event.artist._offsets3d
        picked_point = (x[ind], y[ind], z[ind])

        # Handle removal of duplicate points
        if picked_points and picked_point == picked_points[-1]:
            return

        picked_points.append(picked_point)
        print(f'Picked Point: {picked_point}')

        if len(picked_points) > 1:
            # Draw a line from the previous point to the present selected point
            prev_point = picked_points[-2]
            line_xs = [prev_point[0], picked_point[0]]
            line_ys = [prev_point[1], picked_point[1]]
            line_zs = [prev_point[2], picked_point[2]]
            ax.plot(line_xs, line_ys, line_zs, color='r')
            fig.canvas.draw_idle()

    # Connect the pick event to the figure
    fig.canvas.mpl_connect('pick_event', onpick)

    # Set picker to True for the scatter plot to enable picking
    scatter.set_picker(True)

    plt.show()

    # Save corner points
    # if len(corners) > 1: del corners[-1] # Remove last duplicate
    # save_data(corners, 'pcl_corners.npy', CALIB_PATH)

def calibrate(points2D=None, points3D=None):
    # Load corresponding points
    folder = os.path.join(PKG_PATH, CALIB_PATH)
    if points2D is None: points2D = np.load(os.path.join(folder, 'img_corners.npy'))
    if points3D is None: points3D = np.load(os.path.join(folder, 'pcl_corners.npy'))
    
    # Check points shape
    assert(points2D.shape[0] == points3D.shape[0])
    if not (points2D.shape[0] >= 5):
        rospy.logwarn('PnP RANSAC Requires minimum 5 points')
        return

    # Obtain camera matrix and distortion coefficients
    camera_matrix = np.array([1345.61962890625, 0.0, 960.0, 0.0, 1345.61962890625, 540.0, 0.0, 0.0, 1.0]).reshape(3, 3)
    dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    # Estimate extrinsics
    success, rotation_vector, translation_vector, inliers = cv2.solvePnPRansac(points3D, 
        points2D, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    
    # Compute re-projection error.
    points2D_reproj = cv2.projectPoints(points3D, rotation_vector,
        translation_vector, camera_matrix, dist_coeffs)[0].squeeze(1)
    assert(points2D_reproj.shape == points2D.shape)
    error = (points2D_reproj - points2D)[inliers].reshape(-1, 2)  # Compute error only over inliers.
    rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
    rospy.loginfo('Re-projection error before LM refinement (RMSE) in px: ' + str(rmse))

    # Refine estimate using LM
    if not success:
        rospy.logwarn('Initial estimation unsuccessful, skipping refinement')
    elif not hasattr(cv2, 'solvePnPRefineLM'):
        rospy.logwarn('solvePnPRefineLM requires OpenCV >= 4.1.1, skipping refinement')
    else:
        assert len(inliers) >= 3, 'LM refinement requires at least 3 inlier points'
        rotation_vector, translation_vector = cv2.solvePnPRefineLM(points3D[inliers],
            points2D[inliers], camera_matrix, dist_coeffs, rotation_vector, translation_vector)
        
        # Compute re-projection error.
        points2D_reproj = cv2.projectPoints(points3D, rotation_vector,
            translation_vector, camera_matrix, dist_coeffs)[0].squeeze(1)
        assert(points2D_reproj.shape == points2D.shape)
        error = (points2D_reproj - points2D)[inliers].reshape(-1, 2)  # Compute error only over inliers.
        rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
        rospy.loginfo('Re-projection error after LM refinement (RMSE) in px: ' + str(rmse))

    # Convert rotation vector
    rotation_matrix = cv2.Rodrigues(rotation_vector)[0]
    euler = euler_from_matrix(rotation_matrix)
    
    # Save extrinsics
    np.savez(os.path.join(folder, 'extrinsics.npz'),
        euler=euler, R=rotation_matrix, T=translation_vector.T)

    # Display results
    print('Euler angles (RPY):', euler)
    print('Rotation Matrix:', rotation_matrix)
    print('Translation Offsets:', translation_vector.T)

def project_point_cloud(velodyne, img_msg, image_pub):
    # Read image using CV bridge
    try:
        img = CV_BRIDGE.imgmsg_to_cv2(img_msg, 'bgr8')
    except CvBridgeError as e: 
        rospy.logerr(e)
        return

    # Transform the point cloud
    try:
        transform = TF_BUFFER.lookup_transform('world', 'velodyne', rospy.Time())
        velodyne = do_transform_cloud(velodyne, transform)
    except tf2_ros.LookupException:
        pass

    # Extract points from message
    points3D = ros_numpy.point_cloud2.pointcloud2_to_array(velodyne)
    points3D = np.asarray(points3D.tolist())
    
    # Group all beams together and pick the first 4 columns for X, Y, Z, intensity.
    if OUSTER_LIDAR: points3D = points3D.reshape(-1, 9)[:, :4]
    
    # Filter points in front of camera
    inrange = np.where((points3D[:, 2] > 0) &
                       (points3D[:, 2] < 6) &
                       (np.abs(points3D[:, 0]) < 6) &
                       (np.abs(points3D[:, 1]) < 6))
    max_intensity = np.max(points3D[:, -1])
    points3D = points3D[inrange[0]]

    # Color map for the points
    cmap = matplotlib.cm.get_cmap('jet')
    colors = cmap(points3D[:, -1] / max_intensity) * 255

    # Project to 2D and filter points within image boundaries
    points2D = [ CAMERA_MODEL.project3dToPixel(point) for point in points3D[:, :3] ]
    points2D = np.asarray(points2D)
    inrange = np.where((points2D[:, 0] >= 0) &
                       (points2D[:, 1] >= 0) &
                       (points2D[:, 0] < img.shape[1]) &
                       (points2D[:, 1] < img.shape[0]))
    points2D = points2D[inrange[0]].round().astype('int')

    # Draw the projected 2D points
    for i in range(len(points2D)):
        cv2.circle(img, tuple(points2D[i]), 2, tuple(colors[i]), -1)

    # Publish the projected points image
    try:
        image_pub.publish(CV_BRIDGE.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError as e: 
        rospy.logerr(e)

if __name__ == '__main__':

    image_color = '/sensors/camera/image_color'
    velodyne_points = '/sensors/velodyne_points'

    bag_file_name = sys.argv[1]

    BAG_FILE_PATH = "src/lidar_camera_calibration/bagfiles"

    bag = rosbag.Bag(os.path.join(BAG_FILE_PATH, bag_file_name))

    img_msg = None
    lidar   = None

    for topic, msg, t in bag.read_messages(topics=[image_color, velodyne_points]):
        if topic == image_color:
            img_msg = msg
        elif topic == velodyne_points:
            lidar = msg

        if img_msg is not None and lidar is not None:
            break

    bag.close()

    extract_points_3D(lidar, time.time())
    extract_points_2D(img_msg, time.time())