#! /usr/bin/env python3
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from arc_planner.msg import BoundingBox, BoundingBoxes
from cv_bridge import CvBridge
import cv2.aruco as aruco
br = CvBridge()

class Img:

    first_img = False

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        print("stuck1")

        self.matrix_coefficients = np.zeros((3,3))
        self.distortion_coefficients = np.zeros((1,5))
        self.image_sub = rospy.Subscriber('/zed2i/zed_node/rgb_raw/image_raw_color',Image,self.image_callback,queue_size = 1) #/compressed
        self.bb_pub = rospy.Publisher('/bb_aruco', BoundingBoxes, queue_size=10)
        self.bboxes = BoundingBoxes()

        while not self.first_img and not rospy.is_shutdown():
            print("stuck")
            pass
        print("Done Initializing Subs")

    def image_callback(self,msg):
        self.frame = br.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        print("Frame: ", self.frame.shape)

        self.first_img = True

    def publish_bb(self):
        frame = self.frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50) #Supposed to be 5x5_250
        parameters = aruco.DetectorParameters_create()

        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters = parameters) #-- works as initial parameters probably - look to line 15,16
        if np.all(ids is not None):
            for i in range(0, len(ids)):
                rvec,tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i],0.02, self.matrix_coefficients, self.distortion_coefficients)
                (rvec - tvec).any()
            aruco.drawDetectedMarkers(frame, corners,ids,(0,0,255))
            # aruco.drawAxis(frame, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec,1.0)


            self.bboxes = BoundingBoxes()

        for i in range(len(corners)):	
            msg = BoundingBox()
            msg.xmin = int(corners[i][0][0][0])
            msg.ymin = int(corners[i][0][0][1])
            msg.xmax = int(corners[i][0][1][0])
            msg.ymax = int(corners[i][0][1][1])
            msg.id = ids[i][0]
            msg.cen_y,msg.cen_x,channel=frame.shape
            self.bboxes.bounding_boxes.append(msg)
            print("Detected: ", abs((msg.xmin - msg.xmax)*(msg.ymin - msg.ymax)))	
				
        if not len(corners):
            msg = BoundingBoxes()
            msg.bounding_boxes=[]
            self.bboxes.bounding_boxes=[]
            print("Not Detected.")
			
        self.bb_pub.publish(self.bboxes)
        print(corners)
        print(frame.shape)
        frame = cv2.circle(frame, (320,180), 5, (255, 0, 0), -1)
        # imS = cv2.resize(frame, (640, 360))
        cv2.imshow('image1',frame)

        key = cv2.waitKey(3)

        
        # Debug block 
        
        # print(corners)
        # print(frame.shape)
        # frame = cv2.circle(frame, (320,240), 5, (0, 0, 255), -1)
        # cv2.imshow('image1',frame)
        # key = cv2.waitKey(3)
        

    def main(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_bb()
            r.sleep()
if __name__ == '__main__':
    print("stuck")
    rospy.init_node('aruco_viewer')
    server = Img()
    server.main()
    rospy.spin()


