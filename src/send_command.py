#!/usr/bin/env python3
"""
A basic ROS command line controller for the Interbotix PX100 Arm. 
Run the program, read the README or code, to find more details about the commands
"""

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String, Float32
import time

class SendCommand():

    def __init__(self):
        # print initial title and notes 
        print("THE COMMAND LINE PX100 ARM CONTROLLER!")
        print("NOTE: The y value is actually moving the waist of the arm which uses a different command than the x and z values which sets the joint position.\n" 
                "The range is (-3.1, 3.1). If you enter the same value twice, the waist will not move because it is already at the set coordinate.")

        # initialize variables 
        self.x = 0 
        self.y = 0
        self.z = 0
        self.exit = "exit"
        self.home = "home"
        self.sleep = "sleep"
        self.open_gripper = "og"
        self.close_gripper = "cg"
        self.open = "open"
        self.close = "close"
        
        # set up publishers for all the different messages that the controller will receive
        self.point_publisher = rospy.Publisher("/arm_control/point", Point, queue_size=1)
        self.pose_publisher = rospy.Publisher("/arm_control/pose", Point, queue_size=1)
        self.home_publisher = rospy.Publisher("/arm_control/home", Bool, queue_size=1)
        self.sleep_publisher = rospy.Publisher("/arm_control/sleep", Bool, queue_size=1)
        self.gripper_publisher = rospy.Publisher("/arm_control/gripper", String, queue_size=1)
        self.exit_publisher = rospy.Publisher("/arm_control/exit", Bool, queue_size=1)
        self.time_publisher = rospy.Publisher("/arm/time", Float32, queue_size=1)
        
        self.publisher()

        # set up the subscriber to get the coordinates form the camera
        self.point_sub = rospy.Subscriber("/cargo_point", Point, queue_size=1)

    def print_commands(self):
        print(">-----------------COMMANDS----------------------<")
        print("Enter a point (x,y,z): p 0 0 0")
        print("Enter ee pose matrix (x,y,z): m 0 0 0")
        print("Set trajectory time: t 1.0")
        print("Go to home position: home")
        print("Go to sleep position: sleep")
        print("Open gripper: og")
        print("Close gripper: cg")
        print("Exit: exit")
        print(">-----------------------------------------------<")

        

    def publisher(self):
        count = 0
        self.print_commands()
        while True: 
            # repeat command info every 5 commands
            if count == 5:
                self.print_commands()
                count = 0
            
            # ask for user input
            user_input = input("Please enter a new command: ")
            user_input = user_input.strip()
            # exit 
            if user_input == self.exit:
                rospy.loginfo("Returning the arm to the sleep position and exiting the program now.")
                self.exit_publisher.publish(True)
                break
            # home
            elif self.home in user_input:
                rospy.loginfo("Moving to home position")
                self.home_publisher.publish(True)
            # sleep
            elif self.sleep in user_input:
                rospy.loginfo("Moving to sleep position")
                self.sleep_publisher.publish(True)
            # gripper open
            elif self.open_gripper in user_input: 
                rospy.loginfo("Opening the gripper")
                self.gripper_publisher.publish(self.open)
            # gripper close
            elif self.close_gripper in user_input:
                rospy.loginfo("Closing the gripper")
                self.gripper_publisher.publish(self.close)
            # move by point amount
            elif user_input[0] == "p":
                user_input = user_input.split(" ")
                can_send = self.check_input(user_input)
                if can_send:
                    self.point_publisher.publish(Point(self.x, self.y, self.z))
            # set ee pose components
            elif user_input[0] == "m":
                user_input = user_input.split(" ")
                can_send = self.check_input(user_input)
                if can_send:
                    self.pose_publisher.publish(Point(self.x, self.y, self.z))
            elif user_input[0] == "t":
                user_input = user_input.split(" ")
                if self.is_dig(user_input[1]):
                    self.time_publisher.publish(float(user_input[1]))

            else: 
                rospy.loginfo("Improper command, please try again.")
            
            count += 1

    def check_input(self,input):
        can_send = True
        if len(input) != 4: 
            rospy.loginfo("Improper command, please try again.")
        else: 
            # check the x value
            if self.is_dig(input[1]):
                self.x = float(input[1])
            else: 
                rospy.loginfo("x={} is not a float".format(self.x))
                can_send = False

            # check the y value
            if self.is_dig(input[2]):
                self.y = float(input[2])
                if self.y < -3.1 or self.y > 3.1:
                    rospy.loginfo("y={} is not within the range (-3.1, 3.1)".format(self.y))
                    can_send = False
            else: 
                rospy.loginfo("y={} is not a float".format(self.y))
                can_send = False
            
            # check the z value
            if self.is_dig(input[3]):
                self.z = float(input[3])
            else: 
                rospy.loginfo("z={} is not a float".format(self.z))
                can_send = False

            if can_send:
                rospy.loginfo("Sending command: ({}, {}, {})".format(self.x, self.y, self.z))
            else: 
                rospy.loginfo("Improper input numbers. Please try again")
            
            return can_send
    
    def is_dig(self,n):
        try:
            float(n)
            return True
        except ValueError:
            return  False
    

if __name__=='__main__':
    rospy.init_node("send_message")
    try:
        SendCommand()
    except rospy.ROSInterruptException:
        pass
