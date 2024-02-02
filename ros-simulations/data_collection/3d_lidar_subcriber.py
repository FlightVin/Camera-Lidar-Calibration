#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2
import rosbag
import sys
import os

class LidarSubscriber:
    def __init__(self, lidar_exp, exp_num, img_num):
        self.lidar_data = None
        self.lidar_sub = rospy.Subscriber('/3d_lidar/depth/points', PointCloud2, self.lidar_callback)
        
        self.lidar_exp = lidar_exp
        self.exp_num = exp_num
        self.img_num = img_num

        rospy.on_shutdown(self.save_data)

    def lidar_callback(self, data):
        rospy.loginfo("Received Lidar data!")
        self.lidar_data = data  

    def save_data(self):
        if self.lidar_data is not None:
            try:

                directory_name = self.lidar_exp + '/' + self.exp_num

                if not os.path.exists(directory_name):
                    os.makedirs(directory_name)
                    print(f"Directory '{directory_name}' created.")
                else:
                    print(f"Directory '{directory_name}' already exists.")

                bag = rosbag.Bag(f'{directory_name}/{self.img_num}.bag', 'w')
                bag.write('/3d_lidar/depth/points', self.lidar_data)
                bag.close()
                rospy.loginfo(f"Bag file saved as {directory_name}/{self.img_num}.png")
            except Exception as e:
                rospy.logerr(e)

def main(idar_exp, exp_num, img_num):
    rospy.init_node('lidar_subscriber', anonymous=True)
    lidar_subscriber = LidarSubscriber(idar_exp, exp_num, img_num)
    rospy.spin()

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
