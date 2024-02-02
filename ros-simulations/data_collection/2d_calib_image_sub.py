#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import sys
import os

class DataCollector:
    def __init__(self, lidar_exp, exp_num, img_num):
        self.image_data = None
        self.lidar_data = None

        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.lidar_sub = rospy.Subscriber('/lidar/scan', LaserScan, self.lidar_callback)
        
        self.bridge = CvBridge()

        self.lidar_exp = lidar_exp
        self.exp_num = exp_num
        self.img_num = img_num

        rospy.on_shutdown(self.save_data)

    def image_callback(self, data):
        rospy.loginfo("Received an image!")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            self.image_data = cv_image
        except Exception as e:
            rospy.logerr(e)

    def lidar_callback(self, data):
        rospy.loginfo("Received Lidar data!")
        # Process LiDAR data here if needed
        self.lidar_data = data

    def save_data(self):
        if self.image_data is not None and self.lidar_data is not None:
            try:
                directory_name = os.path.join(self.lidar_exp, self.exp_num)

                if not os.path.exists(directory_name):
                    os.makedirs(directory_name)
                    rospy.loginfo(f"Directory '{directory_name}' created.")
                else:
                    rospy.loginfo(f"Directory '{directory_name}' already exists.")

                # Save image
                cv2.imwrite(f'{directory_name}/{self.img_num}.png', self.image_data)
                rospy.loginfo(f"Image saved as {directory_name}/{self.img_num}.png")

                # Process and save LiDAR data if needed
                # Example: Save LiDAR data to a file
                with open(f'{directory_name}/{self.img_num}_lidar.txt', 'w') as file:
                    file.write(str(self.lidar_data))

                rospy.loginfo(f"LiDAR data saved as {directory_name}/{self.img_num}_lidar.txt")
            except Exception as e:
                rospy.logerr(e)

def main(lidar_exp, exp_num, img_num):
    rospy.init_node('data_collector', anonymous=True)
    data_collector = DataCollector(lidar_exp, exp_num, img_num)
    rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Usage: python script_name.py <lidar_exp> <exp_num> <img_num>")
        sys.exit(1)
    
    lidar_exp, exp_num, img_num = sys.argv[1:]
    rospy.loginfo(f"Lidar Exp: {lidar_exp}, Experiment Number: {exp_num}, Image Number: {img_num}")

    main(lidar_exp, exp_num, img_num)


# #!/usr/bin/python3

# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import sys
# import os

# class ImageSubscriber:
#     def __init__(self, lidar_exp, exp_num, img_num):
#         self.image_data = None
#         self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
#         self.bridge = CvBridge()

#         self.lidar_exp = lidar_exp
#         self.exp_num = exp_num
#         self.img_num = img_num

#         rospy.on_shutdown(self.save_image)

#     def image_callback(self, data):
#         rospy.loginfo("Received an image!")
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
#             self.image_data = cv_image
#         except Exception as e:
#             rospy.logerr(e)

#     def save_image(self):
#         if self.image_data is not None:
#             try:

#                 directory_name = self.lidar_exp + '/' + self.exp_num

#                 if not os.path.exists(directory_name):
#                     os.makedirs(directory_name)
#                     print(f"Directory '{directory_name}' created.")
#                 else:
#                     print(f"Directory '{directory_name}' already exists.")

#                 cv2.imwrite(f'{directory_name}/{self.img_num}.png', self.image_data)
#                 rospy.loginfo(f"Image saved as {directory_name}/{self.img_num}.png")
#             except Exception as e:
#                 rospy.logerr(e)

# def main(lidar_exp, exp_num, img_num):
#     rospy.init_node('image_subscriber', anonymous=True)
#     image_subscriber = ImageSubscriber(lidar_exp, exp_num, img_num)
#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         lidar_exp, exp_num, img_num = sys.argv[1:]
#     except:
#         raise ValueError("Expected more args")
    
#     print(f"""
#     Lidar Exp: {lidar_exp}
#     Experiment Number: {exp_num}
#     Image Number: {img_num}    
#     """)

#     main(lidar_exp, exp_num, img_num)
