#! /usr/bin/env python3
import rospy 			
import math			
import numpy as np	
import utm
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32
import matplotlib.pyplot as plt

class RootClient(object):
    start_lat=0
    start_lon=0
    def __init__(self):

        print("[.] Initializing Subscriber and GPS")
       
        self.odom_sub = rospy.Subscriber('/useful_pose',PoseWithCovarianceStamped, self.odom_cb)
        self.origin_odom_sub = rospy.Subscriber('/mavros/global_position/local',Odometry, self.origin_odom_cb)

        print("hellooo")
        self.cur_east = 0.0
        self.cur_north = 0.0
        self.gps_cnt = 0
        self.set_gps = False
        self.first_odom=False
        while not self.first_odom and not rospy.is_shutdown():
            print("bruh")
            continue        
        print("[+] Done initializing Subscriber")

    def odom_cb(self,msg):
        print("haha")
        self.cur_east=msg.pose.pose.position.x
        self.cur_north=msg.pose.pose.position.y
        self.first_odom=True    
    def origin_odom_cb(self,msg):
        self.origin_x=msg.pose.pose.position.x
        self.origin_y=msg.pose.pose.position.y
    def main(self):

        r = rospy.Rate(10)
        self.start_east = self.cur_east
        self.start_north = self.cur_north
        while not rospy.is_shutdown():
            plt.axis("equal")            
            # plt.scatter([self.cur_east], [self.cur_north], label = "Current Position", color = "cyan")
            plt.plot(self.cur_east,self.cur_north,".g")
            plt.plot(self.origin_x,self.origin_y,".b")

            plt.pause(0.001)
            r.sleep()
        
                    
if __name__ == "__main__":

    rospy.init_node("GPS_Waypoint_Visualizer")
    root = RootClient()
    print("All set!")
    root.main()
    rospy.spin()
    rospy.logwarn("Killing!")


