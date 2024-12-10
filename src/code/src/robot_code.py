#! /usr/bin/env python

import argparse
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import matplotlib.pyplot as plt

import time
import os
import tf
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
import rospy
import intera_interface
from PIL import Image as PILImage
from PIL import ImageDraw

from client import get_centroid_and_recyclable_label


FIRST_IMG = None

class RobotCode:
    def __init__(self):
        rospy.init_node('robot_code', anonymous=True)
        self.bridge = CvBridge()

        #self.cv_color_image = None
        self.cv_depth_image = None

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)

        self.first_image = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.tf_listener = tf.TransformListener()  # Create a TransformListener object

        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)     # gettting camera info for pixel conversion

        self.point_pub = rospy.Publisher("goal_point", Point, queue_size=10)
        self.image_pub = rospy.Publisher('detected_cup', Image, queue_size=10)

        
        
        rospy.spin()
    
    def camera_info_callback(self, msg):
        # Extracting the intrinsic parameters from the CameraInfo message (look this message type up online)
        K = msg.K
        self.fx = K[0]
        self.fy = K[4]
        self.cx = K[2]
        self.cy = K[5]
    
    def pixel_to_point(self, u, v, depth):
        # Use the camera intrinsics to convert pixel coordinates to real-world coordinates
        X = ((u-self.cx)*depth)/self.fx
        Y = ((v-self.cy)*depth)/self.fy
        Z = depth
        print("p2p")
        print((X, Y, Z))
        return X, Y, Z

    def image_callback(self, img_data):
        """The callback function to show image by using CvBridge and cv
        """
        # (edge_detection, window_name) = ignore

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_data, "bgr8")
            img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            if self.first_image is None:
                self.first_image = img_rgb
                print('got image')
                self.centroid, self.label = get_centroid_and_recyclable_label(self.first_image)
                # self.centroid[1] = abs(self.centroid[1])
                print(type(self.centroid[0]))
                print("got centroid, got label!")

                # plot centroid on image and save down
                if self.centroid and self.label:
                    pilimg = PILImage.fromarray(self.first_image)
                    draw = ImageDraw.Draw(pilimg)
                    point_radius = 5
                    point_color=(255,0,0)
                    x,y = self.centroid
                    draw.ellipse(
                        [(x-point_radius, y-point_radius), (x+point_radius, y+point_radius)],
                        fill=point_color,
                        outline=point_color
                    )
                    pilimg.save('plottedcentroid1.png')
                    print("img saved!")
            # If we have both color and depth images, process them
            if self.cv_depth_image is not None:
                self.process_images()
            

        except CvBridgeError as err:
            rospy.logerr(err)
            return
    
    def depth_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (16UC1 format)
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

        except Exception as e:
            print("Error - depth:", e)
    
    def process_images(self):

        # Fetch the depth value at the center
        center_x, center_y = self.centroid
        depth = self.cv_depth_image[ int(center_y), int(center_x)]
        print(self.fx)
        print(self.fy)
        print(self.cx)
        print(self.cy)
        if self.fx and self.fy and self.cx and self.cy:
            camera_x, camera_y, camera_z = self.pixel_to_point(center_x, center_y, depth)
            
            camera_link_x, camera_link_y, camera_link_z = camera_z, -camera_x, -camera_y
            # Convert from mm to m
            camera_link_x /= 1000
            camera_link_y /= 1000
            camera_link_z /= 1000

            # Convert the (X, Y, Z) coordinates from camera frame to base frame
            # TODO: is base the right frame we're trying to transform to??
            try:
                self.tf_listener.waitForTransform("/base", "/camera_link", rospy.Time(), rospy.Duration(10.0))
                point_base = self.tf_listener.transformPoint("/base", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/camera_link"), point=Point(camera_link_x, camera_link_y, camera_link_z)))
                # 0.1 x 0 y 0.03 z
                X_base, Y_base, Z_base = point_base.point.x, point_base.point.y, point_base.point.z+0.06
                print("Real-world coordinates in base frame: (X, Y, Z) = ({:.2f}m, {:.2f}m, {:.2f}m)".format(X_base, Y_base, Z_base))

                if X_base < 0.001 and X_base > -0.001:
                    print("Erroneous goal point, not publishing - Is the cup too close to the camera?")
                else:
                    print("Publishing goal point: ", X_base, Y_base, Z_base)
                    # Publish the transformed point
                    self.point_pub.publish(Point(X_base, Y_base, Z_base))

                    cup_img = self.first_image.copy()
                    
                    # Convert to ROS Image message and publish
                    ros_image = self.bridge.cv2_to_imgmsg(cup_img, "bgr8")
                    self.image_pub.publish(ros_image)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("TF Error: " + str(e))
                return



if __name__ == '__main__':
    RobotCode()
        # def run():
    #     # rp = intera_interface.RobotParams()
    #     # valid_cameras = rp.get_camera_names()
    #     # if not valid_cameras:
    #     #     rp.log_message(("Cannot detect any camera_config"
    #     #         " parameters on this robot. Exiting."), "ERROR")

    #     #     return

    #     # while not FIRST_IMG:
    #     #     print("Waiting for image ...")
    #     #     time.sleep(1)

    #         if FIRST_IMG:
    #             pil_image = PILImage.fromarray(FIRST_IMG)
    #             pil_image.save('pilimg.jpg')
    #             print('image found!')
    #             break

    #         rate.sleep()

        # centroid = get_centroid(image)