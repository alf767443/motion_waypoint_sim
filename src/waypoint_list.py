#!/usr/bin/env python3

# Import library
import rospy, os
import pandas as pd

# Import the waypoint_list.csv.
class ReadCSV():
    # Init the ros node
    def __init__(self, file_path=os.path.dirname(os.path.abspath(__file__))+'/waypoint_list.csv'):
        rospy.init_node('waypoint_list')
        rospy.loginfo('Initing the waypoint read from: ' + file_path)
        # Define the file path
        self.file_path = file_path
        # Read the csv file
        self.read_csv()
        #Keep node alive
        rospy.spin()
    # Read the csv file
    def read_csv(self):
        try:
            self.df = pd.read_csv(self.file_path)
            rospy.loginfo(f"Imported csv with:\n {str(self.df.info())}")
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
            rospy.loginfo(f"The value in row {str(row)} are:\n {str(value)}")
            return value
        except Exception as e:
            rospy.logerr(f"Error to get the row {str(row)}")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False

if __name__ == '__main__':
    try:
        # Run the node and keep it running
        ReadCSV()
    except rospy.ROSInterruptException:
        pass