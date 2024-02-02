import rosbag
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import pandas as pd
from pyntcloud import PyntCloud
import sys

def viz_pointcloud(bag_file):

    print("Visualizing bag file:", bag_file)

    bag = rosbag.Bag(bag_file, "r")

    for topic, msg, _ in bag.read_messages():
        if topic == "/sensors/velodyne_points":
            cloud_generator = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points = np.array(list(cloud_generator))

            theta = np.radians(0) 
            rotation_matrix = np.array([
                [np.cos(theta), 0, np.sin(theta)],
                [0, 1, 0],
                [-np.sin(theta), 0, np.cos(theta)]
            ])

            rotated_points = np.dot(points, rotation_matrix.T)

            plane_points = rotated_points[np.where(rotated_points[:, 1] < 0.146)[0]]

            # rotated_df = pd.DataFrame(data=rotated_points, columns=["x", "y", "z"])
            plane_df = pd.DataFrame(data=plane_points, columns=["x", "y", "z"])

            cloud = PyntCloud(plane_df)

            cloud.plot()

    bag.close()

def main(lidar_exp, exp_num, img_num):
    viz_pointcloud(lidar_exp + '/' + exp_num + '/' + img_num + '.bag')

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
