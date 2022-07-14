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
from tf.transformations import euler_from_quaternion
from math import sqrt
from tf import TransformListener


class Search():

    # create messages that are used to publish feedback/result
    _feedback = arc_planner.msg.SearchTagFeedback()
    _result = arc_planner.msg.SearchTagResult()
    
    # required class_name
    req_class_name = "tag"

    # detected the class
    detected = False

    # iterations
    iterations = 0

    # when to stop searching for the object (in meters)
    stop_iterations = 2000

    # if the callback is the first callback
    first_image = False
    first_box = False

    # image data
    image = np.zeros((1080,1920))

    # bounding box data
    bounding_boxes = []

    # noraml velocities
    linear_vel = 0.1
    angular_vel = 0.2 #0.15 prev

    # spiral velocities
    linear_spiral = 0.2 #0.3
    angular_spiral = 0.15 #0.2

    # pub message
    command = Twist()

    # box vars    
    cx, cy, xmin, ymin, xmax, ymax = 0, 0, 0, 0, 0, 0

    # center of the image
    center = 960

    def __init__(self, name):
        self.bridge = cv_bridge.CvBridge()

        # initialize subscribers
        self.box_sub = rospy.Subscriber('/bb_aruco',BoundingBoxes, self.box_callback)
        self.image_sub = rospy.Subscriber('/zed2i/zed_node/rgb_raw/image_raw_color',Image, self.image_callback) 
        
        while not (self.first_image and self.first_box) and not rospy.is_shutdown():
            print("bru")
            pass

        # initialize publishers
        self.pub_vel = rospy.Publisher('/rover', Twist, queue_size=10)
        self.pub_led = rospy.Publisher('/led', String, queue_size = 10)
        msg = String()
        msg.data = 'red'
        self.pub_led.publish(msg)

        # start the server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, arc_planner.msg.SearchTagAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        print("Done initializing action")

    def box_callback(self,msg):
        self.bounding_boxes = msg.bounding_boxes
        self.first_box = True
    
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.first_image = True

    def spiral(self):
        msg = Twist()
        msg.linear.x = self.linear_spiral
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_spiral
        return msg
    
    def rotate(self, rotation):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        # msg.angular.z = (rotation/320) * self.angular_vel
        msg.angular.z = rotation* self.angular_vel
        return msg

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        return msg

    def tag_detection(self):
        bb = self.bounding_boxes

        mx_area = 0
        cx = 0
        cy = 0

        if len(bb)>0:
            print(self.detected)

            for box in bb:
                #take cx and cy from topic. Consider msg=[x_min,y_min,x_max,y_max]
                xmin=box.xmin 
                xmax=box.xmax
                ymin=box.ymin    
                ymax=box.ymax
                self.center=box.cen_x/2
                area = abs((xmax-xmin)*(ymax-ymin))
                print(area)
                if area<20:
                    return False, cx, cy
                if area>mx_area:
                    cx = (int)((xmin+xmax)/2)
                    cy = (int)((ymin+ymax)/2)
                    mx_area = area
                    
            print(mx_area)
            return True, cx, cy

        return False, cx, cy

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
                
                if area>mx_area:
                    self.xmin = xmin
                    self.xmax = xmax
                    self.ymin = ymin
                    self.ymax = ymax
                    self.cx = (int)((xmin+xmax)/2)
                    self.cy = (int)((ymin+ymax)/2)
                    mx_area = area
                break ## changed
            self.area = mx_area

    def execute_cb(self,goal):
        self.detected = False
        self.bounding_boxes = []
        self.req_class_name = goal.class_name
        self.cx = 0
        self.cy = 0
        r = rospy.Rate(10)

        flag = False
        i = 0
        while self.detected == False and not rospy.is_shutdown():
            i+=1
            if (i%50 == 0):
                self.angular_spiral -= 0.02
                self.iterations +=1

            tag_detect, tag_cx, tag_cy = self.tag_detection()
           
            if tag_detect == True:
                self.detected = True
                self.cx = tag_cx
                self.cy = tag_cy
                flag = True

            if self.iterations > self.stop_iterations:
                flag = True

            if not flag:
                self.command = self.spiral()
            else:
                self.command = self.stop()
                print("Stoppp")
            self.pub_vel.publish(self.command)

            if flag:
                break
            r.sleep()
            # self._feedback.iterations = self.iterations        
            # self._as.publish_feedback(self._feedback)

        if self.detected == True:

            print("Rotating towards the tag before going into follow_tag")
            for i in range(30):
                self.command = self.stop()
                self.pub_vel.publish(self.command)
                r.sleep()
            rospy.sleep(2)

            while(abs(self.cx - self.center) > 20 and not rospy.is_shutdown()):
                self.get_cx_cy()
                error = float(self.center - self.cx)
                # print(error)
                print("Tag Center:",self.cx,"| Error : ",error," center:",self.center)
                rotation = 1 if (error>0) else -1
                self.command = self.rotate(rotation)
                self.pub_vel.publish(self.command)
                r.sleep()
            
            for i in range(10):
                self.command = self.stop()
                self.pub_vel.publish(self.command)
                r.sleep()
            
            self.get_cx_cy() # just to finally update the cx and cy values

            print("Completed rotation, going to follow tag")

            self._result.found = True
            self._result.cx = self.cx
            self._result.cy = self.cy
            self._as.set_succeeded(self._result, 'Found')
            

        else:
            self._result.found = False
            self._as.set_succeeded(self._result, 'Not Found')

if __name__ == '__main__':
    rospy.init_node('search_tag')
    server = Search(rospy.get_name())
    rospy.spin()  
