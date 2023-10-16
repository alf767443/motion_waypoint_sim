#!/usr/bin/env python3

# Import library
import rospy
import pandas as pd

# Import the waypoint_list.csv and convert.
class ReadCSV_Waypoint_List():
    # Init the ros node
    def __init__(self):
        rospy.init_node('waypoint_list')
        rospy.loginfo('Initing the waypoint read from: ' + file_path)
        # Define the file path
        self.file_path = file_path
        # Read the csv file
        self.read_csv()
    
    def get_current_goal():
        rospy.