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
        rospy.Subscriber('/move_base/current_goal', PoseStamped, self.callback_move_base_current_goal, queue_size=10)
        # Subscribe to the 'move_base/status' topic
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.callback_move_base_status, queue_size=10)
        # Subscribe to the 'move_base/goal' topic
        rospy.Subscriber('/move_base/goal', PoseStamped, self.callback_move_base_goal, queue_size=10)


        # Create a publisher to send a goal        
        self.publisher_move_base_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        # Define the variables of control the goal
        self.current_goal_is_seted, self.current_goal_PoseStamped, self.current_goal_status, self.current_goal_seq, self.current_goal_delta_time = False, None, None, None, None
        #Read the csv waypoint_list
        self.wp_list = ReadCSV() 

        self.run_waypoint_list()

        rospy.spin()

    # This function converts the csv dictionary into a PoseStamped message
    def pose_csv_dict2msg(self, input:dict):
        try:
            rospy.logdebug(f"Try to convert the dict\n{str(input)}\nto a PoseStamped message")
            # Create the PoseStamped message
            output = PoseStamped()
            # Preencha o campo 'header' do objetivo
            output.header.frame_id = 'map'
            # Coordinates of the target position (x, y, z)
            output.pose.position = Point(input.pos_x, input.pos_y, input.pos_z)
            # Orientation (quaternion) of the target position (x, y, z, w)
            output.pose.orientation = Quaternion(input.ori_x, input.ori_y, input.ori_z, input.ori_w)
            return output
        except Exception as e:
            rospy.logerr(f"Erro on the dict convert to PoseStamped goal")
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
            self.current_goal_is_seted, self.current_goal_PoseStamped, self.current_goal_status = False, goal, None

        except Exception as e:
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False
    
    # Callback function for the 'move_base/current_goal' topic
    def callback_move_base_current_goal(self, msg):
        rospy.logdebug(f"{msg}")
        # Check if the current_goal of move_base is the equal to the current_goal of motion_waypoint_sim
        if self.current_goal_PoseStamped.pose == msg.pose:
            rospy.logdebug(f"The goal correspond")
            self.current_goal_is_seted, self.current_goal_PoseStamped, self.current_goal_status, self.current_goal_delta_time, self.current_goal_seq = True, msg, None, 0, msg.header.seq
        else:
            rospy.logwarn(f"The goal responded to isn't the one submitted")
            self.current_goal_is_seted = False

    # Check the result of the 'move_base/status' topic
    def callback_move_base_status(self, msg):
        # Check if have a current_goal_seq
        if not type(self.current_goal_seq) is int:
            return False    
        # Check for all values of array to seq number
        for status in msg.status_list:
            # Search for the goal_id that match with current_goal_seq+1
            if f"-{self.current_goal_seq+1}-" in status.goal_id.id:
                # Get status value
                self.current_goal_status = status.status
                # Get the delta time
                self.current_goal_delta_time = msg.header.stamp.secs - status.goal_id.stamp.secs
                return True
        return False

    # Check the result of the 'move_base/goal' topic
    def callback_move_base_goal(self, msg):
        # Check for all values of array to seq number
        self.current_goal_seq = msg.header.seq
        print(self.current_goal_seq)
        return True
        
    # Create a new goal from a goal dict
    def new_goal(self, goal:dict, max_try=10):
        try:
            # Try for a max of 10 times send the goal
            for n_try in range(max_try):
                rospy.loginfo(f"Trying to send the goal\t{n_try+1}/{max_try}")
                rospy.logdebug(f"Creating a new goal from a dict")
                # Convert goal to PoseStamped
                goal_msg = self.pose_csv_dict2msg(input=goal)
                # Send goal to the /move_base/goal topic
                self.send_goal2topic(goal=goal_msg)
                # Wait for move_base/current_goal... Timeout in 5 seconds
                for _ in range(50):
                    if self.current_goal_is_seted:
                        rospy.loginfo(f"A new goal is define like:\n{str(self.current_goal_PoseStamped)}")
                        return True
                    rospy.sleep(0.1)
                rospy.logwarn(f"Timeout of response /move_base/current_goal")
        except Exception as e:
            rospy.logerr(f"An error occurs on create a new goal")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False

    
    def check_status(self):
        # Switch case for the status
        status = self.current_goal_status
        if status == None:
            return True 
        # PENDING=0
        if status == 0:
            return True
        # ACTIVE=1
        elif status == 1:
            return True
        # PREEMPTED=2
        elif status == 2:
            return True
        # SUCCEEDED=3 -> Go to next waypoint
        elif status == 3:
            return False
        # ABORTED=4
        elif status == 4:
            raise AssertionError("The goal is aborted")
        # REJECTED=5
        elif status == 5:
            raise AssertionError("The goal is rejected")
        # PREEMPTING=6
        elif status == 6:
            return True
        # RECALLING=7
        elif status == 7:
            raise AssertionError("The goal is recalling")
        # RECALLED=8
        elif status == 8:
            return True
        # LOST=9
        elif status == 9:
            raise AssertionError("The goal is lost")
        # No match
        else:
            rospy.logwarn(f"The goal status isn't mapped")
            return True


    # Run all waypoints of the list    
    def run_waypoint_list(self, max_wait_to_reached = 600):
        # Run this topic to all waypoints in list
        wp_n_rows = self.wp_list.get_n_rows()
        wp_n = 0
        MAX_TRY = 10
        # Send all waypoints in the csv
        while wp_n < wp_n_rows:
            # Try for the waypoint for a max MAX_TRY try times
            for i in range(MAX_TRY):   
                try:
                    # Settiing the waypoint wp_n from csv
                    rospy.loginfo(f"Setting the waypoint goal {wp_n+1}/{wp_n_rows}")
                    # Get the way point of the row
                    wp = self.wp_list.get_row(row=wp_n)
                    # Create the new goal from wp, else go to next goal
                    if not self.new_goal(goal=wp, max_try=MAX_TRY):
                        raise AttributeError("Error to set the goal")

                    while self.check_status():
                        if self.current_goal_delta_time > max_wait_to_reached:
                            raise TimeoutError("Goal reach timeout")
                        rospy.sleep(0.5)
                    break
                    
                # Other errors
                except (AssertionError, TimeoutError) as e:
                    rospy.logwarn(f"{e}... Try again {i+1}/{MAX_TRY}")
                    continue
                # Erros that continue to next goal
                except AttributeError as e:
                    rospy.logerr(f"{e}")
                    break
            rospy.loginfo(f"Next goal")
            wp_n += 1

        rospy.loginfo(f"End of waypoints")


if __name__ == '__main__':
    try:
        # Run the node and keep it running
        ReadCSV_Waypoint_List()
    except rospy.ROSInterruptException:
        pass

