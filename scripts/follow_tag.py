#! /usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy as np
import arc_planner.msg
import actionlib
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from arc_planner.msg import BoundingBoxes
from math import sqrt

class Follow():
    # create messages that are used to publish feedback/result
    _feedback = arc_planner.msg.FollowTagFeedback()
    _result = arc_planner.msg.FollowTagResult()
    
   # image data
    image = np.zeros((1080,1920))

    # center of the image
    center = 320

    # if the callback is the first callback
    first_image = False
    first_box = False

    # pub message
    command = Twist()

    # bounding box params
    cx, cy, xmin, ymin, xmax, ymax = 0, 0, 0, 0, 0, 0

    # bounding box data
    bounding_boxes = []

    # threshold to stop rotating
    yaw_threshold = 80

    # linear velocity and angular velocity
    linear_vel = 0.15  # 0.3 previosuly
    angular_vel = 0.15 # 0.2 previously

    # area params
    area = 0
    area_threshold = 1000

    def __init__(self, name):
        self.bridge = cv_bridge.CvBridge()

        # initialize subscribers
        self.box_sub = rospy.Subscriber('/bb_aruco', BoundingBoxes, self.box_callback)
        self.image_sub = rospy.Subscriber('/zed2i/zed_node/rgb_raw/image_raw_color', Image, self.image_callback)

        while not self.first_image and not rospy.is_shutdown():
            pass

        # initialize publishers
        self.pub_vel = rospy.Publisher('/rover', Twist, queue_size=10)
        self.pub_led = rospy.Publisher('/led', String, queue_size = 10)
        
        # start the server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, arc_planner.msg.FollowTagAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        print("Done initializing follow tag action")

    def box_callback(self,msg):
        self.bounding_boxes = msg.bounding_boxes
        self.first_box = True
    
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.first_image = True
        # print(self.first_image)

    def get_cx_cy(self):
        bb = self.bounding_boxes

        mx_area = 0
        cx = 0
        cy = 0

        if len(bb)>0:
            for box in bb:
                #take cx and cy from topic. Consider msg=[x_min,y_min,x_max,y_max]
                xmin=box.xmin 
                xmax=box.xmax
                ymin=box.ymin    
                ymax=box.ymax

                area = abs((xmax-xmin)*(ymax-ymin))
                self.center=box.cen_x/2
                
                if area>mx_area:
                    self.xmin = xmin
                    self.xmax = xmax
                    self.ymin = ymin
                    self.ymax = ymax
                    self.cx = (int)((xmin+xmax)/2)
                    self.cy = (int)((ymin+ymax)/2)
                    mx_area = area
            self.area = mx_area
    
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        return msg
    
    def move_forward(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        return msg

    def rotate_with_linear(self, forward, proportion):
        msg = Twist()
        msg.linear.x = self.linear_vel if forward else 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = proportion * self.angular_vel
        return msg

    def execute_cb(self,goal):
        self.cx = goal.cx
        self.cy = goal.cy
        self.area = 0
        self.bounding_boxes = []
        r = rospy.Rate(10)

        while (self.area < self.area_threshold) and not rospy.is_shutdown():

            self.get_cx_cy()
            print(abs(self.cx - self.center))
            if(abs(self.cx - self.center) > 20):
                error = float(self.center - self.cx)
                self.command = self.rotate_with_linear(True, float(error/160))
            
            else:
                self.command = self.move_forward()

            self.pub_vel.publish(self.command)
            r.sleep()
        
        if self.area > self.area_threshold:
            self.command = self.stop()
            self.pub_vel.publish(self.command)
            self._result.reached = True
            self._as.set_succeeded(self._result, 'Reached within 2m of the tag')
            for i in range(250):
                msg = String()
                msg.data = 'green'
                self.pub_led.publish(msg)
                r.sleep()

        else:
            self._result.reached = False
            self._as.set_succeeded(self._result, 'Not reached')

if __name__ == '__main__':
    rospy.init_node('follow_tag')
    server = Follow(rospy.get_name())
    rospy.spin()  
