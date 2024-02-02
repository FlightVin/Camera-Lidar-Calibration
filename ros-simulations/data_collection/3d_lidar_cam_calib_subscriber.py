#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2
import rosbag
import sys
import os

class LidarCamSubscriber:
    def __init__(self, lidar_exp, exp_num, img_num):
        self.lidar_data = None
        self.lidar_sub = rospy.Subscriber('/3d_lidar/depth/points', PointCloud2, self.lidar_callback)

        self.image_data = None
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()

        self.cam_info = None
        self.cam_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.cam_info_callback)

        self.lidar_exp = lidar_exp
        self.exp_num = exp_num
        self.img_num = img_num

        rospy.on_shutdown(self.save_data)

    def lidar_callback(self, data):
        rospy.loginfo("Received Lidar data!")
        self.lidar_data = data  

    def image_callback(self, data):
        rospy.loginfo("Received an image!")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            self.image_data = cv_image
        except Exception as e:
            rospy.logerr(e)

    def cam_info_callback(self, data):
        rospy.loginfo("Received camera info!")
        self.cam_info = data

    def save_data(self):
        if self.image_data is not None and self.lidar_data is not None:
            try:
                directory_name = self.lidar_exp + '/' + self.exp_num

                if not os.path.exists(directory_name):
                    os.makedirs(directory_name)
                    print(f"Directory '{directory_name}' created.")
                else:
                    print(f"Directory '{directory_name}' already exists.")

                cv2.imwrite(f'{directory_name}/{self.img_num}.png', self.image_data)
                rospy.loginfo(f"Image saved as {directory_name}/{self.img_num}.png")

                bag = rosbag.Bag(f'{directory_name}/{self.img_num}.bag', 'w')
                bag.write('/sensors/velodyne_points', self.lidar_data)
                
                image_msg = self.bridge.cv2_to_imgmsg(self.image_data, encoding="bgr8")
                bag.write('/sensors/camera/image_color', image_msg)
                
                bag.write('/sensors/camera/camera_info', self.cam_info)
                bag.close()
                rospy.loginfo(f"Bag file saved as {directory_name}/{self.img_num}.png")
            except Exception as e:
                rospy.logerr(e)

def main(lidar_exp, exp_num, img_num):
    rospy.init_node('lidar_cam_subscriber', anonymous=True)
    lidar_cam_subscriber = LidarCamSubscriber(lidar_exp, exp_num, img_num)
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
