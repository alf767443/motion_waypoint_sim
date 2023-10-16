#!/usr/bin/env python3

# Import library
import rospy
import pandas as pd
from waypoint_list import ReadCSV

# Import messages
from move_base_msgs.msg import *
from geometry_msgs.msg import *

# Import the waypoint_list.csv and convert.
class ReadCSV_Waypoint_List():
    # Init the ROS node
    def __init__(self):
        rospy.init_node('waypoint_manager')
        # Subscribe to the 'move_base/current_goal' topic
        rospy.Subscriber('/move_base/current_goal', PoseStamped, self.move_base_current_goal_callback)
        self.Publisher_move_base_goal = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        
        #Init the waypoint_lis.py
        self.WPL = ReadCSV()        
        self.set_goal({'a':1})


        rospy.spin()
        
    # Callback function for the 'move_base/current_goal' topic
    def move_base_current_goal_callback(self, msg):
        self.current_goal = msg.pose.position
        print(f"{str(msg)}")

    # Callback function for the 'move_base/result' topic
    def set_goal(self, goal:dict):
        rospy.loginfo(self.WPL.get_row(row=1))
        # self.current_goal = msg.pose.position
        # print(f"{str(msg)}")


if __name__ == '__main__':
    try:
        # Run the node and keep it running
        ReadCSV_Waypoint_List()
    except rospy.ROSInterruptException:
        pass
