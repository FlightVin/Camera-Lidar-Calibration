import rosbag
from sensor_msgs.msg import LaserScan
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

def extract_pointcloud(bag_file):
    print("Extracting point cloud from bag file:", bag_file)
    bag = rosbag.Bag(bag_file, "r")

    points = []

    for topic, msg, _ in bag.read_messages():
        if topic == "/lidar/scan":
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            ranges = msg.ranges

            for i, r in enumerate(ranges):
                if np.isfinite(r):
                    angle = angle_min + i * angle_increment
                    x = r * np.cos(angle)
                    y = r * np.sin(angle)
                    points.append([x, y, 0]) 

    bag.close()
    return np.array(points)

def visualize_pointcloud(pointcloud):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(pointcloud[:, 0], pointcloud[:, 1], pointcloud[:, 2], s=1, c='b')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('LiDAR Point Cloud Visualization')

    plt.show()

def main(lidar_exp, exp_num, img_num):
    pointcloud = extract_pointcloud(lidar_exp + '/' + exp_num + '/' + img_num + '.bag')
    visualize_pointcloud(pointcloud)

if __name__ == '__main__':
    try:
        lidar_exp, exp_num, img_num = sys.argv[1:]
    except:
        raise ValueError("Expected more args")
    
    print(f"""
    Lidar Exp: {lidar_exp}
    Experiment Number: {exp_num}
    Image Number: {img_num}    
    """)

    main(lidar_exp, exp_num, img_num)
