#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32

class Odometer:
    rover_pose = PoseWithCovarianceStamped()
    first_pose=False
    gps_satellites_count=0
    def __init__(self):
        #self.zed_odom_sub = rospy.Subscriber("/useful_odomtery", Odometry, self.zed_odom_callback)
        self.zed_pose_sub = rospy.Subscriber("/useful_pose", PoseWithCovarianceStamped, self.zed_pose_callback)
        self.gps_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, self.gps_odom_callback)
        print("gps sub active")
        self.gps_satellites = rospy.Subscriber("/mavros/global_position/raw/satellites", UInt32, self.gps_satellites_callback)
        print("satellite_sub_active")
        self.pose_pub = rospy.Publisher("/really_useful_pose", PoseWithCovarianceStamped, queue_size=1)
        while not rospy.is_shutdown() and not self.first_pose:
            pass
        

    def zed_odom_callback(self,msg):
        self.zed_odom = msg
    def zed_pose_callback(self,msg):
        self.zed_pose = msg
        self.first_pose = True
    def gps_odom_callback(self,msg):
        self.gps_odom = msg
        self.first_pose = True

    def gps_satellites_callback(self,msg):
        print("satelliting")
        self.gps_satellites_count = msg.data
        self.first_pose = True
    
    def main(self, rate):
        if self.gps_satellites_count > 6:
            self.rover_pose.pose = self.gps_odom.pose
            self.rover_pose.header = self.gps_odom.header
        else:
            self.rover_pose = self.zed_pose
        print("publishin")
        self.pose_pub.publish(self.rover_pose)
        self.gps_satellites_count = 0
        rate.sleep()

        

if __name__ == '__main__':
    rospy.init_node("poser")
    server = Odometer()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        server.main(rate)
    rospy.spin()
