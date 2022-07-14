#! /usr/bin/env python3
import rospy
import numpy as np
import arc_planner.msg
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi, sqrt, atan2


class Planner():
    
    # action variables
    _feedback = arc_planner.msg.BugFeedback()
    _result = arc_planner.msg.BugResult()
    
    previous_x = 0.0
    previous_y = 0.0
    cur_x = 0.0
    cur_y = 0.0
    goal_x = 6.0
    goal_y = 0.0

    # orientation
    rover_theta = 0.0
    goal_theta = 0.0

    # distance travelled 
    total_distance = 0.0

    # ranges
    ranges = [float('inf')]*180
    #scan_range = 20
    scan_range = 179
    range_idx = 0

    prev_min = float('inf')
    cur_min = float('inf')

    tangent_diff = 0.0
    tangent_threshold = 1.0
    clear_threshold = 1.0 # distance to move forward after rotating

    # subscriber bools
    odom_first = False
    scan_first = False

    # publisher msgs
    command = Twist()

    # errors
    err_dist = 0.0
    err_angle = 0.0

    # thresholds
    yaw_threshold = pi/60   # previous pi/36
    dist_threshold = 0.5

    # distance from the object to stop (in meters)
    depth_threshold = 1.3

    # linear velocity and angular velocity
    linear_vel = 0.3  # 0.3 previosuly
    angular_vel = 0.2 # 0.2 previously

    # target bools
    reached = False

    def __init__(self, name):

        # define goal
        # self.goal_x = goal_x
        # self.goal_y = goal_y

        # initialize subscribers
        #self.odom_sub = rospy.Subscriber('/zed2i/zed_node/odom', Odometry, self.odom_callback)
        self.pose_sub = rospy.Subscriber('/zed2i/zed_node/pose_with_covariance', PoseWithCovarianceStamped, self.odom_callback)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        while not self.odom_first and not rospy.is_shutdown():
            continue
        while not self.scan_first and not rospy.is_shutdown():
            continue
        print("[+] Subs initialized")

        # initialize publishers
        self.pub_vel = rospy.Publisher('/rover', Twist, queue_size=10)
        #first callback
        self.first = False

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, arc_planner.msg.BugAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        print('[+] Done initializing server, pubs, subs')

    def odom_callback(self, msg):
        if not self.odom_first:
            self.previous_x = msg.pose.pose.position.x
            self.previous_y = msg.pose.pose.position.y 
            self.odom_first = True

        # localize
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y

        # orient
        rot_q = msg.pose.pose.orientation
        ( _, _, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        if theta<0:
            theta+=2*pi
        self.rover_theta = theta

        # orient goal
        inc_x = self.goal_x - self.cur_x
        inc_y = self.goal_y - self.cur_y

        self.goal_theta = atan2(abs(inc_y), abs(inc_x))
        if inc_x > 0 and inc_y < 0:
            self.goal_theta = 2*pi - self.goal_theta
        
        elif inc_x < 0 and inc_y > 0:
            self.goal_theta = pi - self.goal_theta
        
        elif inc_x < 0 and inc_y < 0:
            self.goal_theta = pi + self.goal_theta

        # calculate errs
        self.err_dist = sqrt(inc_x**2 + inc_y**2)
        err_angle = self.goal_theta - self.rover_theta
        if err_angle > pi:
            self.err_angle = err_angle - 2*pi
        elif err_angle < -pi:
            self.err_angle = 2*pi + err_angle
        else:
            self.err_angle = err_angle

        # calculate and update distance travelled till now
        d_increment = sqrt((self.cur_x - self.previous_x)**2 + (self.cur_y - self.previous_y)**2)
        self.total_distance = self.total_distance + d_increment        
        self.previous_x = self.cur_x
        self.previous_y = self.cur_y

    def scan_callback(self, msg):
        temp_ranges = list(msg.ranges)
        # temp_ranges = temp_ranges[int(-self.scan_range/2):] + temp_ranges[:int(self.scan_range/2)]
        # temp_ranges = temp_ranges[(int(self.scan_range/2)-70):(int(self.scan_range/2)+70)]
        temp_ranges.reverse()
        self.ranges = temp_ranges

        self.cur_min = min(self.ranges)

        if not self.scan_first:
            self.prev_min = self.cur_min

        self.tangent_diff = self.cur_min - self.prev_min
        self.prev_min = self.cur_min
        self.scan_first = True
        # if not rospy.is_shutdown():
        #    print(temp_ranges)

    def move_forward(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        return msg

    def rotate(self, rotation):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
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

    def rotate_with_linear(self, forward, proportion):
        msg = Twist()
        msg.linear.x = self.linear_vel if forward else 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = proportion * self.angular_vel
        return msg

    def align_goal(self):
        r = rospy.Rate(10)
        
        print()
        print("=====")
        print("Rotating towards the goal before going into move_goal")
        while( abs(self.err_angle) > self.yaw_threshold and not rospy.is_shutdown()):

            rotation = 1 if (self.err_angle>0) else -1
            self.command = self.rotate(rotation)
            self.pub_vel.publish(self.command)
            r.sleep()
        
        self.command = self.stop()
        self.pub_vel.publish(self.command)
        rospy.sleep(0.5)

    def move_goal(self):
        r = rospy.Rate(10)
        flag = True

        print()
        print("=====")
        print("Moving towards the goal")
        while (self.err_dist > self.dist_threshold) and not rospy.is_shutdown():
            print(str(self.err_dist)+"\r")
            if min(self.ranges) < self.depth_threshold:
                self.range_idx = self.ranges.index(min(self.ranges))
                flag = False
                break

            if(abs(self.err_angle) > pi/60):
                self.command = self.rotate_with_linear(True, float(self.err_angle/2*pi))
            
            else:
                self.command = self.move_forward()

            self.pub_vel.publish(self.command)
            r.sleep()

        self.command = self.stop()
        self.pub_vel.publish(self.command)

        return flag
    
    def plan(self):
        r = rospy.Rate(10)
        flag = False
        self.total_distance = 0.0

        while not flag:
            print()
            print("=====")
            print("Obstacle Detected!\nMoving away from the obstacle")
            rotation = -1 if (self.range_idx < int(self.scan_range/2)) else 1

            print("Rotating!")
            while( (self.tangent_diff < self.tangent_threshold) and not rospy.is_shutdown()):

                self.command = self.rotate(rotation)
                self.pub_vel.publish(self.command)
                r.sleep()
            rospy.sleep(0.75)        ## changed
            
            self.command = self.stop()
            self.pub_vel.publish(self.command)
            rospy.sleep(1)

            self.total_distance = 0.0   #CHANGED
            print("Path clear, moving forward")
            while (self.total_distance < self.clear_threshold) and not rospy.is_shutdown():

                if min(self.ranges) < self.depth_threshold:
                    flag = True
                    break
                self.command = self.move_forward()
                self.pub_vel.publish(self.command)
                r.sleep()

            self.command = self.stop()
            self.pub_vel.publish(self.command)

            if not flag:
                break
            else:
                flag = not flag

        print("Algining towards goal again!")

    def execute_cb(self,goal):

        self.goal_x = goal.goal_x
        self.goal_y = goal.goal_y
        self.reached = False

        while not self.reached:
            self.align_goal()
            self.reached = self.move_goal()
            
            if not self.reached:
                self.plan()
        
        # TODO
        # When do you give up on the current goal point and return false?
        
        self._result.reached = True
        self._as.set_succeeded(self._result, 'Reached within 2m of the marker')
        print("Testing over, go home :)")

if __name__ == '__main__':
    rospy.init_node('planner')
    server = Planner(rospy.get_name())
    rospy.spin()  
