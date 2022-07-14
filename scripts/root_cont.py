#! /usr/bin/env python3
import rospy 		
import sys			
import os			
import math			
import numpy as np	
import utm
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist 
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32
import actionlib
import arc_planner.msg
from arc_planner.srv import save_photo, save_photoRequest, save_photoResponse
class RootClient(object):

    def __init__(self):

        print("Initializing Subscriber/Publishers/Actions/Services/Variables")
        """
        Initialize all ROS services, actions and Pub/Sub
        Actions to be called --> follow_tag, search_tag, temp_planner #data collector and data transmitter actions need to be written
        Subscribed Topics --> odom (not needed)
        """

        # class variables initialized
        self.reached = False
        self.cx = 0
        self.cy = 0

        # current position of the bot - dont need
        self.cur_x = 0.0
        self.cur_y = 0.0

        # subscriber initialize
        self.first_odom_clbk=False
        self.set_gps = False
        self.gps_cnt = 0
        self.start_lat = 0.0
        self.start_lon = 0.0

        #self.odom_sub = rospy.Subscriber('/zed2i/zed_node/odom', Odometry , self.odom_cb)
        self.odom_sub = rospy.Subscriber('/zed2i/zed_node/odom', Odometry , self.odom_cb)
        # self.odom_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_cb)

        while not self.first_odom_clbk and not rospy.is_shutdown():
            continue

        # while not self.set_gps and not rospy.is_shutdown():
        #     continue

        # actions
        self.search_tag_client = actionlib.SimpleActionClient('search_tag', arc_planner.msg.SearchTagAction)
        self.search_tag_client.wait_for_server()
        print("[+] initialized spiral search")

        self.follow_tag_client = actionlib.SimpleActionClient('follow_tag', arc_planner.msg.FollowTagAction) 
        self.follow_tag_client.wait_for_server()
        print("[+] initialized follow tag")

        self.planner_client = actionlib.SimpleActionClient('planner', arc_planner.msg.BugAction) 
        self.planner_client.wait_for_server()
        print("[+] initialized planner")

        print("[+] Done initializing Subscriber/Publishers/Actions/Services/Variables.")
        self.image_service = rospy.ServiceProxy('save_image', save_photo) # Defines a service proxy for calling the A* service
        try:
            print("Waiting for services") # I mean its a print statement dont worry
            rospy.wait_for_service('save_image') # this might worry you tbh, but lite, it just waits for servie till ist up and running (haa haa naam hi repeat kar raha hu thik hai)
            print("Done waiting for services")
        except rospy.ServiceException as e: # try catch block stuff, its called error handling, standard stuff, check this to get help: Docs: https://docs.python.org/3/tutorial/errors.html for video enthusiasts: https://www.youtube.com/watch?v=NIWwJbo-9_8
            rospy.logerr("Services could not be initialized")



    def odom_cb(self, msg):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        self.first_odom_clbk=True 
    
    def gps_cb(self, msg):
        self.gps_cnt += 1
        if self.gps_cnt <= 100:
            self.start_lat = msg.latitude
            self.start_lon = msg.longitude
            print(f"GPS set at {self.start_lat}, {self.start_lon}")
        else:
            self.set_gps = True


    def search_cb(self,fb_data):
        pass

    def move_fb(self,fb_data):
        pass

    def main(self, goal_x, goal_y, astro):

        """
        Main controller function. 
        1. Call planner_client to move to all 5 waypoints (A,B,C,D,E and F)
        2. For points D and E call search tag and follow tag

        To-DO:
        1. Write collect_data and transmit_data actions to be called at D and E after reaching tag.
        """

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.astro = astro
        self.reached = False

        print(float(self.goal_x), float(self.goal_y))
        #Creates a goal to send to the action server.
        self.planner_goal = arc_planner.msg.BugGoal(goal_x=float(self.goal_x), goal_y=float(self.goal_y))
		
        while(not self.reached and not rospy.is_shutdown()):
            #Sends the goal to the action server.
            self.planner_client.send_goal(self.planner_goal, feedback_cb = self.move_fb) #Send goal to Bug planner
            self.planner_client.wait_for_result()

            #Result of executing the action
            status = self.planner_client.get_result() 
            if status.reached == True:
                print("Successfully reached the goal point!")
                self.reached = True
            else:
                print("Failed to reach marker.")

        if self.astro:
            #Search for the astronaut
            self.search_goal = arc_planner.msg.SearchTagGoal(class_name="tag")
            self.search_tag_client.send_goal(self.search_goal, feedback_cb=self.search_cb)
            self.search_tag_client.wait_for_result()

            self.found = self.search_tag_client.get_result().found
            self.cx = self.search_tag_client.get_result().cx
            self.cy = self.search_tag_client.get_result().cy

            if self.found == False:
                print("Failed to find the Astronaut.")
                return self.status
            else:
                print("Found the Astronaut.")
                        
            #Reach the Astronaut
            self.follow_tag_goal = arc_planner.msg.FollowTagGoal(cx = self.cx, cy = self.cy)
            self.follow_tag_client.send_goal(self.follow_tag_goal)
            self.follow_tag_client.wait_for_result()

            if self.follow_tag_client.get_result().reached == False:
                print("Failed to reach the Astronaut")
                return self.status
            else:
                print("Reached the Astronaut")
            #Call the action to collect bluetooth data
            #Call the action to trasmit collected data to astronaut
            try:        
                response = self.image_service(True) # Sending the map, start_pos and goal_pos to A* service, which will then return a path as a trajectory, ie a list of Point()s, IKR!!! Majik!!!
                print(response.ack) # A* service response, ack means acknowledge btw... its a bool variable which defines wheter A* was successful.
                if response.ack: # if A* is successful first we say YAY!!! and then assign its responses to lists defined above
                    print("YAY!!!!!")
                else: # if it fails we say NOOO!! and raise up self.err and break out of the entire root controller (IK Sadge right)
                    print("NOOOOo!!!!")
            except rospy.ServiceException as e: # Error handling again but this one lite, you can mostly do nothing abt it (unless if u are careless enough to devide by a 0 in the service you know... ofc not speaking from experience)
                print("Service call failed:" + str(e))
                self.err = True
        self.status = True
        return self.status
        
                    
if __name__ == "__main__":

    rospy.init_node("controller_client")
    root = RootClient()
    print("All set!")
    words=[]
    goal_x = []
    goal_y = []

    co_file = open("/home/atharvmane/catkin_ws/src/arc_planner/scripts/root_cont_coordinates.txt","r")
    lines = co_file.readlines()

    start_lat = root.start_lat
    start_lon = root.start_lon

    start_east, start_nort, start_reg, start_reg_char=utm.from_latlon(start_lat, start_lon)
    # for line in lines:
    #     easting,northing,region_num,region_char=utm.from_latlon(float(line.split()[0]), float(line.split()[1]))
    #     goal_x.append(float(easting-start_east))
    #     goal_y.append(float(northing-start_nort))

    for line in lines:
        goal_x.append(float(line.split()[0]))
        goal_y.append(float(line.split()[1]))
    co_file.close()
    print(goal_x)
    print(goal_y)

    for i in range(4): #Assuming order of points is B,C,D,E,F and A
        if i not in [2]:
            status = root.main(goal_x[i], goal_y[i],False)
        else:
            status = root.main(goal_x[i], goal_y[i],True)
        #if status == True:
        #    print(f"Cleared requirement of point {i+1}!")
        #else:
        #    print(f"Something went wrong with GNSS co-ordinate {i+1}, planning back.")
        #    i-=1
    rospy.spin()
    rospy.logwarn("Killing!")