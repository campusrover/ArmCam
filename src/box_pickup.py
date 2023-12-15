#!/usr/bin/env python3
"""
A basic ROS command line controller for the Interbotix PX100 Arm. 
Run the program, read the README or code, to find more details about the commands
"""

import rospy, math
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String, Float32



class SendCommand():

	def __init__(self):

		# Coordinates to be sent to the arm
		self.x = 0 
		self.y = 0
		self.z = 0

		self.arm_x=0
		self.arm_y=0

		self.current_x=0
		self.current_y=0
		self.current_z=0

		self.at_height=False

		self.old_box_x=0
		self.old_box_y=0

		self.state="waiting"
		

		# Cargo's physical coordinates
		self.cargo_point_sub = rospy.Subscriber("cargo_point", Point, self.cargo_point_cb)
		self.arm_point_sub=rospy.Subscriber("arm_point", Point, self.arm_point_cb)

		# Coordinate with the transportation robot
		self.alien_state_sub = rospy.Subscriber("alien_state", Bool, self.set_state)
		self.alien_state = False

		self.arm_status_publisher = rospy.Publisher("/arm_status", String, queue_size=1)

		# Send commands to arm
		self.point_publisher = rospy.Publisher("/arm_control/point", Point, queue_size=1)
		self.home_publisher = rospy.Publisher("/arm_control/home", Bool, queue_size=1)
		self.sleep_publisher = rospy.Publisher("/arm_control/sleep", Bool, queue_size=1)
		self.gripper_publisher = rospy.Publisher("/arm_control/gripper", String, queue_size=1)
	
	# True if robot has finished moving to loading zone and False otherwise
	def set_state(self, msg):
		self.alien_state = msg.data

	# Drop cargo to preset destination zone
	def drop_cargo(self):
			self.point_publisher.publish(Point(0, -1.2, 0))
			rospy.sleep(2)
			self.point_publisher.publish(Point(0, -1.2, -0.08))
			rospy.sleep(3)
			self.point_publisher.publish(Point(-.08, -1.2, 0))
			rospy.sleep(3)
			self.gripper_publisher.publish("open")
			rospy.sleep(3)
			self.point_publisher.publish(Point(.08, -1.2, 0))
			rospy.sleep(3)
			self.point_publisher.publish(Point(0, -1.2, .1))
			rospy.sleep(3)
			self.home_publisher.publish(True)
			rospy.sleep(2)
			self.sleep_publisher.publish(True)
			rospy.sleep(2)
			self.gripper_publisher.publish("close")
			self.arm_status_publisher.publish("resting")



	def arm_point_cb(self,msg):
		self.arm_x=msg.x
		self.arm_y=msg.y

		

	# Calculate the location of the Cargo based on the sent location.
	def cargo_point_cb(self, msg):
		if(self.at_height==False and self.alien_state==True and self.state=="waiting"):
			self.state="setting_height"
			if(self.old_box_x==0):
				self.old_box_x=msg.x
				self.old_box_y=msg.y

			self.home_publisher.publish(True)
			rospy.sleep(4)
			self.gripper_publisher.publish("open")
			rospy.sleep(1.5)
			self.point_publisher.publish(Point(0,0,-0.1))
			rospy.sleep(4)
			self.point_publisher.publish(Point(-0.05,0,0))
			rospy.sleep(4)



			self.at_height=True
			self.state="moving"

		if (self.alien_state and self.state=="moving"):	# Calculate only if robot is in position

			if(self.arm_y!=0 and self.arm_x!=0):
				
				flags=0
				# distance=math.sqrt( ((self.arm_x-self.old_box_x)**2)+((self.arm_y-self.old_box_x)**2))
				# print("Distance: "+str(distance))
				# print("X: "+str(self.old_box_x-self.arm_x))
				# print("Y: "+str(y-self.arm_y))
				
				if (abs(self.old_box_x-self.arm_x)<=6):
					flags+=1
				elif(self.old_box_x>self.arm_x):
					self.current_y-=0.02
				elif(self.old_box_x<self.arm_x):
					self.current_y+=0.02

				
				if (abs((self.old_box_y)-self.arm_y)<=10):
					self.current_x=0
					flags+=1
				elif(self.old_box_y>self.arm_y):
					self.current_x-=0.01
				elif(self.old_box_y<self.arm_y):
					self.current_x+=0.01

				if(flags==2):
					self.state="descending"
				else:
					self.point_publisher.publish(Point(self.current_x,self.current_y,0))
					self.current_x=0
					rospy.sleep(2)

		elif self.state=="descending" and self.alien_state:
			if(abs(self.current_y>0.2)):
				if(self.current_y<0):
					self.current_y+=0.03
				else:
					self.current_y-=0.03
			self.point_publisher.publish(Point(0,self.current_y,-0.01))
			rospy.sleep(5)

			self.point_publisher.publish(Point(0,self.current_y,-0.03))
			rospy.sleep(5)

			
			self.gripper_publisher.publish("close")
			rospy.sleep(1.5)
			self.sleep_publisher.publish(True)
			self.alien_state=False
			self.at_height=False
			self.current_y=0
			self.alien_state = False 

			self.old_box_x=0
			self.old_box_y=0
			self.state="waiting"





rospy.init_node("box_pickup")
commander = SendCommand()
rospy.spin()