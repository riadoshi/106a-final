#! /usr/bin/env python

import argparse
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import time
import rospy
import intera_interface
from PIL import Image as PILImage

from client import get_centroid_and_recyclable_label

FIRST_IMG = None

class RobotCode:
    def __init__(self):
        rospy.init_node('robot_code', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/io/internal_camera/head_camera/image_raw", Image, self.image_callback)
        self.first_image = None

        rospy.spin()

    def image_callback(self, img_data):
        """The callback function to show image by using CvBridge and cv
        """
        # (edge_detection, window_name) = ignore

        print('here')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_data, "bgr8")
            img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            if not self.first_image:
                self.first_image = img_rgb
                print('got image')
                self.centroid, self.label = get_centroid_and_recyclable_label(self.first_image)
                print("got centroid, got label!")

        except CvBridgeError as err:
            rospy.logerr(err)
            return

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


if __name__ == '__main__':
    RobotCode()
    