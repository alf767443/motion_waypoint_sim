#!/usr/bin/env python3

# Import library
import rospy, os
import pandas as pd

# Import the waypoint_list.csv.
class ReadCSV():
    # Init the ros node
    def __init__(self, file_path=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))+'/waypoints/waypoint_list.csv'):
        rospy.loginfo('Initing the waypoint read from: ' + file_path)
        # Define the file path
        self.file_path = file_path
        # Read the csv file
        self.read_csv()
        #Keep node alive
    # Read the csv file
    def read_csv(self):
        try:
            self.df = pd.read_csv(self.file_path)
            info = str(self.df.describe())
            rospy.loginfo(f"Imported csv with the description:\n{info}")
            return True
        except Exception as e:
            rospy.logerr(f"Error reading the CSV file: {str(e)}")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False
    # Get a row informarion  
    def get_row(self, row:int):
        try:
            # Get the row value
            value = self.df.iloc[row - 1]
            rospy.logdebug(f"The value in row {str(row)} are:\n {str(value)}")
            return value
        except Exception as e:
            rospy.logerr(f"Error to get the row {str(row)}")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False
    def get_rows(self):
        try:
            # Get the rows array
            rows = self.df.iloc
            return rows
        except Exception as e:
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False
    # Get the number of waypoints
    def get_n_rows(self)->int:
        try:
            n_rows = self.df.shape[0]
            rospy.logdebug(f"The list have {n_rows} of waypoiints")
            return n_rows
        except Exception as e:
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)