#! /usr/bin/env python3
import rospy
import numpy as np
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from std_msgs.msg import String,Float64,UInt32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from math import pi, sqrt, atan2,cos,sin,degrees
import copy

class Odomer:

    gps_odom_first=False
    zed_odom_first=False
    compass_first=False
    satellites=0
    gps_odom=Odometry()
    zed_odom=Odometry()
    zed_pose=PoseWithCovarianceStamped()
    pose_angle_error=0
    odom_angle_error=0
    gps_theta=0
    zed_odom_theta=0
    zed_pose_theta=0
    corrected_odom=Odometry()
    corrected_pose=PoseWithCovarianceStamped()
    gps_count=0
    
    
    def __init__(self):

        self.gps_odom_sub = rospy.Subscriber('/mavros/global_position/local', Odometry, self.gps_odom_callback)
        self.zed_odom_sub = rospy.Subscriber('/zed2i/zed_node/odom', Odometry, self.zed_odom_callback)
        self.zed_pose_sub = rospy.Subscriber('/zed2i/zed_node/pose_with_covariance', PoseWithCovarianceStamped, self.zed_pose_callback)
        # self.satellite_sub = rospy.Subscriber('/mavros/global_position/raw/satellites', UInt32, self.satellite_callback)
        print("[+] Subs initialized")

        # initialize publishers
        self.pub_odom = rospy.Publisher('/useful_odomtery', Odometry, queue_size=10)
        self.pub_pose = rospy.Publisher('/useful_pose', PoseWithCovarianceStamped, queue_size=10)
        self.rate=rospy.Rate(10)


    
    def gps_odom_callback(self,msg):
        # print("hello")
        self.gps_odom = msg
        self.corrected_odom = msg
        rot_q = msg.pose.pose.orientation
        (_,_,self.gps_theta)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        # print(self.gps_theta)
        self.gps_count+=1


    def zed_odom_callback(self,msg):

        self.zed_odom=msg
        self.corrected_odom=msg
        rot_q= msg.pose.pose.orientation
        (_,_,self.zed_odom_theta)=euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])


    def zed_pose_callback(self, msg):

        self.zed_pose=msg
        rot_q= msg.pose.pose.orientation
        (_,_,self.zed_pose_theta)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


    def main(self):
        
        if self.gps_count<50:
            print("\n"+str(self.gps_count))
            self.pose_angle_error = self.gps_theta-self.zed_odom_theta
            self.odom_angle_error = self.gps_theta-self.zed_pose_theta
            if self.pose_angle_error<-pi:
                self.pose_angle_error+=2*pi
            elif self.pose_angle_error>pi:
                self.pose_angle_error-=2*pi
            if self.odom_angle_error<-pi:
                self.odom_angle_error+=2*pi
            elif self.odom_angle_error>pi:
                self.odom_angle_error-=2*pi
            print(str(degrees(self.gps_theta))+"-"+str(degrees(self.zed_odom_theta))+"="+str(degrees(self.odom_angle_error)))
            print(str(degrees(self.gps_theta))+"-"+str(degrees(self.zed_pose_theta))+"="+str(degrees(self.pose_angle_error)))
        
        self.corrected_pose = copy.deepcopy(self.zed_pose)
        self.corrected_pose.pose.pose.orientation=self.gps_odom.pose.pose.orientation
        self.corrected_odom.pose.pose.orientation=self.gps_odom.pose.pose.orientation



        # self.corrected_odom.pose.pose.position.x=self.zed_odom.pose.pose.position.x*cos(self.odom_angle_error)-self.zed_odom.pose.pose.position.y*sin(self.odom_angle_error)
        # self.corrected_odom.pose.pose.position.y=self.zed_odom.pose.pose.position.y*cos(self.odom_angle_error)+self.zed_odom.pose.pose.position.x*sin(self.odom_angle_error)
        self.corrected_odom.pose.pose.position.x=self.zed_odom.pose.pose.position.x
        self.corrected_odom.pose.pose.position.y=self.zed_odom.pose.pose.position.y


        self.corrected_pose.pose.pose.position.x=self.zed_pose.pose.pose.position.x*cos(self.pose_angle_error)-self.zed_pose.pose.pose.position.y*sin(self.pose_angle_error)
        self.corrected_pose.pose.pose.position.y=self.zed_pose.pose.pose.position.y*cos(self.pose_angle_error)+self.zed_pose.pose.pose.position.x*sin(self.pose_angle_error)
        
        
        
        # self.corrected_pose.pose.pose.position.x=self.zed_pose.pose.pose.position.x
        # self.corrected_pose.pose.pose.position.y=self.zed_pose.pose.pose.position.y

        self.pub_odom.publish(self.corrected_odom)
        self.pub_pose.publish(self.corrected_pose)
        print(str(degrees(self.gps_theta))+"-"+str(degrees(self.zed_odom_theta))+"="+str(degrees(self.odom_angle_error))+", "+str(self.zed_odom.pose.pose.position.x-self.gps_odom.pose.pose.position.x), end="\r")

        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('odometer')
    server = Odomer()
    while not rospy.is_shutdown():
        server.main()
    rospy.spin() 
