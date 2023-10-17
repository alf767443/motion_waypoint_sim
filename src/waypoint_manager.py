#!/usr/bin/env python3

# Import library
import rospy
import pandas as pd
from waypoint_list import ReadCSV

# Import messages
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *

# Import the waypoint_list.csv and convert.
class ReadCSV_Waypoint_List():
    # Init the ROS node
    def __init__(self):
        rospy.init_node('waypoint_manager')
        # Subscribe to the 'move_base/current_goal' topic
        # rospy.Subscriber('/move_base/current_goal', PoseStamped, self.move_base_current_goal_callback)

        # Create a publisher to send a goal        
        self.publisher_move_base_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        
        #Read the csv waypoint_list
        self.wp_list = ReadCSV() 
        # Global variables
        self.goal_received = False
        
        self.new_goal(self.get_goal_from_list(order=1))

        rospy.spin()

    # This function converts the csv dictionary into a MoveBaseActionGoal message
    def pose_csv_dict2msg(self, input:dict):
        try:
            rospy.logdebug(f"Try to convert the dict\n{str(input)}\nto a MoveBaseActionGoal message")
            # Create the MoveBaseActionGoal message
            output = PoseStamped()
            # Preencha o campo 'header' do objetivo
            output.header.frame_id = 'map'
            # Coordinates of the target position (x, y, z)
            output.pose.position = Point(input.pos_x, input.pos_y, input.pos_z)
            # Orientation (quaternion) of the target position (x, y, z, w)
            output.pose.orientation = Quaternion(input.ori_x, input.ori_y, input.ori_z, input.ori_w)
            return output
        except Exception as e:
            rospy.logerr(f"Erro on the dict convert to MoveBaseActionGoal goal")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False

    # Get goal from list with the wp_list class
    def get_goal_from_list(self, order:int):
        try:
            rospy.logdebug(f"Getting the goal {str(order)}\n from the csv list")
            # Get the goal from list
            goal_dict = self.wp_list.get_row(order)
            return goal_dict
        except Exception as e:
            rospy.logerr(f"Error on get item {str(order)} from the csv list")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False
    
    # Send a goal to the topic /move_base/goal
    def send_goal2topic(self, goal:type):
        try:
            rospy.logdebug(f"Publishing to the publisher_move_base_goal a new goal")
            # Try to send goal to the /move_base/goal
            self.publisher_move_base_goal.publish(goal)
            print(goal)
            
        except Exception as e:
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False
        
    # Create a new goal from a goal dict
    def new_goal(self, goal:dict, max_try=10):
        try:
            self.goal_received = False
            # Try for a max of 10 times send the goal
            for n_try in range(1,max_try):
                rospy.logdebug(f"Creating a new goal from a dict")
                # Convert goal to MoveBaseActionGoal
                goal_msg = self.pose_csv_dict2msg(input=goal)
                # Send goal to the /move_base/goal topic
                self.send_goal2topic(goal=goal_msg)
                try:
                    # Wait for the response goal
                    msg = rospy.wait_for_message('/move_base/current_goal', PoseStamped, timeout=1)
                    # Check if the goal is correct
                    if goal_msg.pose == msg.pose:
                        rospy.loginfo(f"A new goal is define to \n {str(goal)}")
                        return True
                    # Not same goal handle
                    else:
                        rospy.logwarn(f"The goal responded to isn't the one submitted")
                # Timeout handle
                except rospy.exceptions.ROSException:
                    rospy.logwarn(f"Timeout of response /move_base/current_goal")
                    pass
                # Try again handle
                rospy.loginfo(f"Trying to send the goal again\t{n_try}/{max_try}")
        except Exception as e:
            rospy.logerr(f"An error occurs on create a new goal")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False
            

        
    # Callback function for the 'move_base/current_goal' topic
    def move_base_current_goal_callback(self, msg):
        self.current_goal = msg.pose.position
        print(f"{str(msg)}")


if __name__ == '__main__':
    try:
        # Run the node and keep it running
        ReadCSV_Waypoint_List()
    except rospy.ROSInterruptException:
        pass
