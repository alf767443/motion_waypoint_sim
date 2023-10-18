#!/usr/bin/env python3

# Import library
import rospy
import pandas as pd
from waypoint_list import ReadCSV

# Import messages
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from actionlib_msgs.msg import *

# Import the waypoint_list.csv and convert.
class ReadCSV_Waypoint_List():
    # Init the ROS node
    def __init__(self):
        rospy.init_node('waypoint_manager')
        # Subscribe to the 'move_base/current_goal' topic
        rospy.Subscriber('/move_base/current_goal', PoseStamped, self.check_current_goal)

        # Create a publisher to send a goal        
        self.publisher_move_base_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.current_goal_is_seted, self.current_goal_pose = False, None
        #Read the csv waypoint_list
        self.wp_list = ReadCSV() 

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
    
    # Send a goal to the topic /move_base_simple/goal
    def send_goal2topic(self, goal:type):
        try:
            rospy.logdebug(f"Publishing to the publisher_move_base_goal a new goal")
            # Try to send goal to the  /move_base_simple/goal
            self.publisher_move_base_goal.publish(goal)
            # Set the global current_goal
            self.current_goal_is_seted, self.current_goal_pose = False, goal.pose
        except Exception as e:
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False
    
    # Callback function for the 'move_base/current_goal' topic
    def check_current_goal(self, msg):
        rospy.logdebug(f"{msg}")
        msg_pose = msg.pose
        # Check if the current_goal of move_base is the equal to the current_goal of motion_waypoint_sim
        if self.current_goal_pose == msg_pose:
            rospy.logdebug(f"The goal correspond")
            self.current_goal_is_seted = True
        else:
            rospy.logwarn(f"The goal responded to isn't the one submitted")
            self.current_goal_is_seted = False

    # Create a new goal from a goal dict
    def new_goal(self, goal:dict, max_try=10):
        try:
            # Try for a max of 10 times send the goal
            for n_try in range(max_try):
                rospy.loginfo(f"Trying to send the goal\t{n_try+1}/{max_try}")
                rospy.logdebug(f"Creating a new goal from a dict")
                # Convert goal to MoveBaseActionGoal
                goal_msg = self.pose_csv_dict2msg(input=goal)
                # Send goal to the /move_base/goal topic
                self.send_goal2topic(goal=goal_msg)
                # Wait for move_base/current_goal... Timeout in 5 seconds
                for i in range(50):
                    if self.current_goal_is_seted:
                        rospy.loginfo(f"A new goal is define to \n{str(goal)}")
                        return True
                    rospy.sleep(0.1)
                rospy.logwarn(f"Timeout of response /move_base/current_goal")
        except Exception as e:
            rospy.logerr(f"An error occurs on create a new goal")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False




    # Run all waypoints of the list    
    def run_waypoint_list(self, max_wait_to_reached = 600):
        # Run this topic to all waypoints in list
        wp_n_rows = self.wp_list.get_n_rows()
        for wp_n in range(wp_n_rows):
            rospy.loginfo(f"Setting the waypoint {wp_n+1}/{wp_n_rows}")
            wp = self.wp_list.get_row(row=wp_n)
            self.new_goal(goal=wp)
            try:
                while True:
                    move_base_status = rospy.wait_for_message('/move_base/status', GoalStatusArray, timeout=2)
                    if move_base_status.status_list.status == 1:
                        delta_time = move_base_status.header.stamp.secs - move_base_status.status_list.goal_id.stamp.secs
                        if delta_time > max_wait_to_reached:
                            raise TimeoutError('Goal reach timeout')
                    elif move_base_status.status_list.status == 3:
                        rospy.loginfo(f"Goal reached... Next goal")
                        break
                    else:
                        rospy.logwarn(f"Status not reconized: {str(move_base_status.status_list)}")

                    rospy.sleep(0.5)
            except rospy.exceptions.ROSException:
                rospy.logerr(f"Timeout of /move_base/status... Finalising tasks")
                break
            except TimeoutError:
                rospy.logerr(f"Timeout to reach the goal... Next goal")
                continue
        rospy.loginfo(f"End of waypoints")


if __name__ == '__main__':
    try:
        # Run the node and keep it running
        ReadCSV_Waypoint_List()
    except rospy.ROSInterruptException:
        pass

