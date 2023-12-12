#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from std_msgs.msg import Bool,String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
# from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

class ImageProcessor:
	
	def __init__(self):
		
		self.upper_mask = numpy.array([ 100, 255, 255 ]) #Set Upper Color of Range Here
		self.lower_mask = numpy.array([ 40, 0, 230 ]) #Set Lower Color of Range Here


		# Get image from the camera
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.process_image)
		
		self.color_sub=rospy.Subscriber("color_array",String,self.set_arrays)

		# Image processing
		self.bridge = cv_bridge.CvBridge()
		self.kernel = numpy.ones((7,7), numpy.uint8)
		
		# Pixel dimensions of image
		width_pixels = rospy.get_param("~width_pixels", 640)
		height_pixels = rospy.get_param("~height_pixels", 480)

		# Physical dimensions captured by image in meters
		width_phys = rospy.get_param("~width_phys", 0.228)
		height_phys = rospy.get_param("~height_phys", 0.181)

		# Convert pixels to physical dimensions
		self.width_ratio = width_phys / width_pixels 
		self.height_ratio = height_phys / height_pixels 

	def set_arrays(self,msg):
		colors_list=msg.data.split("-")
		for i in range(len(colors_list)):
			colors_list[i]=colors_list[i].split(",")
			for j in range(len(colors_list[i])):
				colors_list[i][j]=int(colors_list[i][j])
		self.upper_mask=numpy.array(colors_list[0])
		self.lower_mask=numpy.array(colors_list[1])

	def process_image(self, msg):		
		# Get image
		image = self.bridge.imgmsg_to_cv2(msg)

		#We dont want the extra area around the arm on the camera, so we mask it with 3 rectanlges
		
		#adding first rectangle (Left Side)
		croppedimage=image
		startpoint=(0,0)
		endpoint=(190,1000)
		color=(0,0,0)
		thickness=-1
		croppedimage=cv2.rectangle(croppedimage,startpoint,endpoint,color,thickness)

		#adding second rectangle (Top)
		croppedimage=image
		startpoint=(0,0)
		endpoint=(1000,130)
		color=(0,0,0)
		thickness=-1
		croppedimage=cv2.rectangle(croppedimage,startpoint,endpoint,color,thickness)

		#adding third rectangle (Right Side)
		croppedimage=image
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

		# If a centroid can be found, the cargo is in view
		if M['m00'] != 0:

			# Compute centroid
			x_pixels = int(M['m10']/M['m00'])
			y_pixels = int(M['m01']/M['m00'])

			# Compute physical x/y coordinates
			x = x_pixels*self.width_ratio
			y = y_pixels*self.height_ratio

			# print(x, y)

			# Publish the position of the cargo box
			# self.cargo_point_pub.publish(Point(x, y, -0.14))#self.arm_z))

		else:
			pass
			# Cargo is not in view, publish an invalid z
			# self.cargo_point_pub.publish(Point(0, 0, 10))

		cv2.imshow("masked image", masked)
		# cv2.imshow("image", croppedimage)
		cv2.waitKey(3)
			

rospy.init_node("color_mask_test_cam")
image_processor = ImageProcessor()
rospy.spin()