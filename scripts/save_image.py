#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from math import sqrt
from arc_planner.srv import save_photo, save_photoResponse
from sensor_msgs.msg import CompressedImage,Image
from cv_bridge import CvBridge

image_num=0
class Imager:
    def __init__(self):
        self.first_img=False
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/zed2i/zed_node/rgb_raw/image_raw_color',Image,self.image_callback,queue_size = 1)
        self.filename="/home/atharvmane/AstroImg"
        self.status=False
        while not self.first_img and not rospy.is_shutdown():
            print("stuck")
            pass
        print("Done Initializing Subs")

    def image_callback(self,msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.first_img = True
    def save_image(self):
        global image_num
        cv2.imwrite(self.filename+str(image_num)+".jpg", self.frame)
        image_num+=1
        self.status=True
        return self.status
def image_saver(request):
    imager=Imager()
    status=imager.save_image()
    return save_photoResponse(status)

def image_server():
    rospy.init_node('save_image')
    s = rospy.Service('save_image', save_photo, image_saver)
    print("Ready to save_image")
    rospy.spin()

if __name__ == "__main__":
    while not rospy.is_shutdown():
        image_server()
    