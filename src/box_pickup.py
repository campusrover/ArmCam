#!/usr/bin/env python3
"""
A basic ROS command line controller for the Interbotix PX100 Arm. 
Run the program, read the README or code, to find more details about the commands
"""

import rospy, numpy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String, Float32

class SendCommand():

	def __init__(self):

		# Coordinates to be sent to the arm
		self.x = 0 
		self.y = 0
		self.z = 0
		
		# Radius conversion to arm command
		self.x_transform_bins = [0.10403, 0.10855, 0.11395, 0.11774, 0.12060, 0.12629, 0.13075, 0.13609]
		self.x_transform_commands = [-.03, -.02, -.01, 0, .01, .02, .03, 0.3]
		
		# Cargo's physical coordinates
		self.cargo_point_sub = rospy.Subscriber("cargo_point", Point, self.cargo_point_cb)

		# Coordinate with the transportation robot
		self.alien_state_sub = rospy.Subscriber("alien_state", Bool, self.set_state)
		self.alien_state = False

		self.arm_status_publisher = rospy.Publisher("/arm_status", String, queue_size=1)

		# Send commands to arm
		self.point_publisher = rospy.Publisher("/arm_control/point", Point, queue_size=1)
		self.home_publisher = rospy.Publisher("/arm_control/home", Bool, queue_size=1)
		self.sleep_publisher = rospy.Publisher("/arm_control/sleep", Bool, queue_size=1)
		self.gripper_publisher = rospy.Publisher("/arm_control/gripper", String, queue_size=1)
	
	# Transform polar coordinates into arm command coordinates
	def trans_x(self, radius):
		return self.x_transform_commands[numpy.searchsorted(self.x_transform_bins, radius)]
	def trans_y(self, theta):
		return 0.1372*theta**3-0.0177*theta**2-0.8057*theta

	# True if robot has finished moving to loading zone and False otherwise
	def set_state(self, msg):
		self.alien_state = msg.data

	# Pick up cargo with given coordinates
	def pickup(self):
			self.arm_status_publisher.publish("grabcube")
			rospy.sleep(1.5)
			self.home_publisher.publish(True)
			rospy.sleep(1.5)
			self.gripper_publisher.publish("open")
			rospy.sleep(1.5)
			self.point_publisher.publish(Point(0, self.y, 0))
			rospy.sleep(3)
			self.point_publisher.publish(Point(self.x, self.y, 0))
			rospy.sleep(3)
			self.point_publisher.publish(Point(0, self.y, self.z))
			rospy.sleep(3)
			self.gripper_publisher.publish("close")
			rospy.sleep(1.5)
			self.point_publisher.publish(Point(0, self.y, -self.z + .01))
			rospy.sleep(1.5)
			self.home_publisher.publish(True)
			rospy.sleep(1.5)

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

	# Return True if arm is capable of picking up the cargo and False otherwise
	def is_valid_coordinate(self, radius, theta):
		if ((radius < .09975 or radius > .13609) or (abs(theta)>0.7) or (self.z == 10)):
			print("invalid coordinates")
			self.arm_status_publisher.publish("invalid")
			return False
		self.arm_status_publisher.publish("valid")
		return True

	# Calculate the 
	def cargo_point_cb(self, msg):

		if (self.alien_state):	# Calculate only if robot is in position
			self.alien_state = False	# Only perform one calculation per cycle
			x = msg.x
			y = msg.y-0.0739083	# Shift so that arm is at (0,0)
			self.z = msg.z      # No conversion necessary

			# Convert to polar
			r = numpy.sqrt(x**2 + y**2)
			t = numpy.arctan2(y, x)

			print(f"radius: {r} theta: {t}")

			# Transform polar coordinates to arm command coordinates
			if (self.is_valid_coordinate(r, t)):
				self.x = self.trans_x(r)
				self.y = self.trans_y(t)
				print(f"original: {self.x}, {self.y}")

				# Adjust for orientation of cube
				if abs(self.y) > 0.01:
					if self.y < 0:
						if self.y > -0.2:
							self.y -= .03
						elif self.y > -0.4:
							self.y -= .09
						else:
							self.y -= .1
					else:
						if self.y < 0.2:
							self.y += .03
						elif self.y < 0.4:
							self.y += .08
						elif self.y < 0.5:
							self.y += .14
						else:
							self.y += .23	
				print(f"adjusted: {self.x}, {self.y}")
				
				self.pickup()
				self.drop_cargo()



rospy.init_node("box_pickup")
commander = SendCommand()
rospy.spin()