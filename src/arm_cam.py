#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
# from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

class ImageProcessor:
	
	def __init__(self):
		#Positions for the arm at the home position, but with a z of -0.14
		#Determined through an upsetting amount of trial and error
		self.arm_initial_x=342
		self.arm_initial_y=207

		self.upper_mask = numpy.array([ 100, 255, 255 ]) #Set Upper Color of Range Here for Main Target
		self.lower_mask = numpy.array([ 65, 0, 230 ]) #Set Lower Color of Range Here for Main Target

		self.upper_mask_tape = numpy.array([ 40, 200, 255 ]) #Set Upper Color of Range Here for tape on Arm
		self.lower_mask_tape = numpy.array([ 1, 100, 10 ])#Set lower Color of Range Here for tape on Arm

		# Get image from the camera
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.process_image)
		
		# State the transportation robot (moving - False, halted - True)
		self.alien_state_sub = rospy.Subscriber("alien_state", Bool, self.set_state)
		self.alien_state = False

		self.sleep_publisher = rospy.Publisher("/arm_control/sleep", Bool, queue_size=1)
		
		self.color_sub=rospy.Subscriber("color_array",String,self.set_arrays)

		# Send position of the cargo
		self.cargo_point_pub = rospy.Publisher("cargo_point", Point, queue_size=10)
		self.arm_point_pub = rospy.Publisher("arm_point", Point, queue_size=10)

		self.arm_status_publisher=rospy.Publisher("arm_status", String, queue_size=1)


		# Image processing
		self.bridge = cv_bridge.CvBridge()
		self.kernel = numpy.ones((7,7), numpy.uint8)
		
		# Pixel dimensions of image
		self.width_pixels = rospy.get_param("~width_pixels", 640)
		self.height_pixels = rospy.get_param("~height_pixels", 480)

		# z distance from arm at "home" position to the box
		self.arm_z = rospy.get_param("~arm_z", -0.14)

	def set_state(self, msg):
		self.alien_state = msg.data

	def set_arrays(self,msg):
		colors_list=msg.data.split("-")
		for i in range(len(colors_list)):
			colors_list[i]=colors_list[i].split(",")
			for j in range(len(colors_list[i])):
				colors_list[i][j]=int(colors_list[i][j])
		self.upper_mask=numpy.array(colors_list[0])
		self.lower_mask=numpy.array(colors_list[1])

	def process_image(self, msg):
		# Only process if the transportation robot is halted
		if (self.alien_state):
			
			# Get image
			image = self.bridge.imgmsg_to_cv2(msg)


			#Second Color Mask for Arm Position:
			hsv2 = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

			mask2 = cv2.inRange(hsv2, self.lower_mask_tape, self.upper_mask_tape)
			
			# Remove noise from mask
			mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, self.kernel)
			mask2= cv2.morphologyEx(mask2, cv2.MORPH_OPEN, self.kernel)

			# Perform filtering
			masked2 = cv2.bitwise_and(image, image, mask=mask2)
			masked2 = cv2.cvtColor(masked2, cv2.COLOR_HSV2BGR)
			masked2 = cv2.cvtColor(masked2, cv2.COLOR_RGB2GRAY)

			# Compute the centroid
			M2 = cv2.moments(masked2)

			x_pixels2=0
			y_pixels2=0
			if M2['m00'] != 0:

				# Compute centroid
				x_pixels2=int(M2["m10"]/M2["m00"])
				y_pixels2 = int(M2['m01']/M2['m00'])
	
				self.arm_point_pub.publish(Point(x_pixels2, y_pixels2, self.arm_z))
			else:
				print("Arm is not in view")

			#We dont want the extra area around the arm on the camera, so we mask it with 3 rectanlges
			#adding first rectangle (Left Side)
			croppedimage=image
			startpoint=(0,0)
			endpoint=(190,1000)
			color=(0,0,0)
			thickness=-1
			croppedimage=cv2.rectangle(croppedimage,startpoint,endpoint,color,thickness)

			#adding second rectangle (Top)
			startpoint=(0,0)
			endpoint=(1000,130)
			color=(0,0,0)
			thickness=-1
			croppedimage=cv2.rectangle(croppedimage,startpoint,endpoint,color,thickness)

			#adding third rectangle (Right Side)
			startpoint=(495,0)
			endpoint=(1000,1000)
			color=(0,0,0)
			thickness=-1
			croppedimage=cv2.rectangle(croppedimage,startpoint,endpoint,color,thickness)

			# Color mask
			hsv = cv2.cvtColor(croppedimage, cv2.COLOR_BGR2HSV)

			mask = cv2.inRange(hsv, self.lower_mask, self.upper_mask)
			
			# Remove noise from mask
			mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
			mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

			# Perform filtering
			masked = cv2.bitwise_and(croppedimage, croppedimage, mask=mask)
			masked = cv2.cvtColor(masked, cv2.COLOR_HSV2BGR)
			masked = cv2.cvtColor(masked, cv2.COLOR_RGB2GRAY)

			# Compute the centroid
			M = cv2.moments(masked)

			x_pixels=0
			y_pixels=0
			# If a centroid can be found, the cargo is in view
			if M['m00'] != 0:

				# Compute centroid
				x_pixels = int(M['m10']/M['m00'])
				y_pixels = int(M['m01']/M['m00'])

				# Publish the position of the cargo box
				self.cargo_point_pub.publish(Point(x_pixels, y_pixels, self.arm_z))

			else:
				# Cargo is not in view, publish an invalid string

				# self.sleep_publisher.publish(True)
				self.arm_status_publisher.publish("invalid")
				# self.alien_state=False

			startpoint=(int(x_pixels2),int(y_pixels2))
			endpoint=(int(x_pixels),int(y_pixels))
			color=(255,0,0)
			thickness=3
			image2=cv2.rectangle(image,startpoint,endpoint,color,thickness)

			cv2.imshow("Increment Image", image2)
			cv2.waitKey(3)


rospy.init_node("arm_cam")
image_processor = ImageProcessor()
rospy.spin()