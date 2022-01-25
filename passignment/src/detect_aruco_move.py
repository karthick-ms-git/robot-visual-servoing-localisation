#!/usr/bin/env	python
# Assignment 
# Detect an aruco marker, find pose of robot, move robot under the marker
# Created by Karthick C K
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import math
import tf
import numpy
import	cv2,	cv_bridge
from	sensor_msgs.msg	import	Image

x = 0.0;y = 0.0 ;theta = 0.0;init=1;first_run=0;init_x = 0.0;init_y = 0.0 
init_theta = 0.0;TR_init=numpy.eye(4);X_Rob=0;Y_Rob=0;angle=0;finish=0

def newOdom(msg):
	global x, y, theta, init, init_x, init_y, init_theta, TR_init

	# Saving initial pose as reference frame 
	if (init==1):
		init_x=msg.pose.pose.position.x
		init_y=msg.pose.pose.position.y
		rot_q = msg.pose.pose.orientation
		(_, _, init_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
		init=0
		T_init= tf.transformations.translation_matrix((numpy.array((msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z))))
		R_init= tf.transformations.quaternion_matrix((numpy.array((rot_q.x, rot_q.y, rot_q.z, rot_q.w))))
		TR_init=numpy.dot(T_init,R_init)
		#print(TR_init)
		#print(numpy.dot(inverse_TR_init,TR_init))
		#print(numpy.round(numpy.dot(inverse_TR_init,TR_init),6))
      
	# Creating relative pose WRT ref frame
	if (init==0):
		rot_q = msg.pose.pose.orientation

		T= tf.transformations.translation_matrix((numpy.array((msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z))))
		R= tf.transformations.quaternion_matrix((numpy.array((rot_q.x, rot_q.y, rot_q.z, rot_q.w))))
		TR=numpy.dot(T,R)
		inverse_TR_init = numpy.linalg.inv(TR_init)
		new_TR=numpy.dot(inverse_TR_init,TR)
		T=tf.transformations.translation_from_matrix(new_TR)
		R=tf.transformations.quaternion_from_matrix(new_TR)
		(_, _, theta_rad) = euler_from_quaternion(R)
		theta=math.degrees(theta_rad)
		x=T[0]
		y=T[1]
		#print(numpy.round(T),math.degrees(theta))
		#print(theta)

def image_callback(msg):
	global image, X_Rob, Y_Rob, angle, finish
	ARUCO_DICT = {"DICT_4X4_50": cv2.aruco.DICT_4X4_50,"DICT_4X4_100": cv2.aruco.DICT_4X4_100,"DICT_4X4_250": cv2.aruco.DICT_4X4_250,"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,"DICT_5X5_50": cv2.aruco.DICT_5X5_50,"DICT_5X5_100": cv2.aruco.DICT_5X5_100,"DICT_5X5_250": cv2.aruco.DICT_5X5_250,"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,"DICT_6X6_50": cv2.aruco.DICT_6X6_50,"DICT_6X6_100": cv2.aruco.DICT_6X6_100,"DICT_6X6_250": cv2.aruco.DICT_6X6_250,"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,"DICT_7X7_50": cv2.aruco.DICT_7X7_50,"DICT_7X7_100": cv2.aruco.DICT_7X7_100,"DICT_7X7_250": cv2.aruco.DICT_7X7_250,"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL}
	image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
	arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_4X4_50"])
	arucoParams = cv2.aruco.DetectorParameters_create()
	(corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
	
	# Finding aruco marker orientation thereby finding robot pose WRT marker
	if len(corners) > 0:
		ids = ids.flatten()
		for (markerCorner, markerID) in zip(corners, ids):
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
			#cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
			#cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
			#cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

			aruco_centre_x = int((topLeft[0] + bottomRight[0]) / 2.0); #print(aruco_centre_x)
			aruco_centre_y = int((topLeft[1] + bottomRight[1]) / 2.0); #print(aruco_centre_y)
			cv2.circle(image, (aruco_centre_x, aruco_centre_y), 4, (0, 0, 255), -1)
			cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
			
			centre_w=948
			centre_h=522
			Y_Rob=(centre_w-aruco_centre_x)*0.004065
			X_Rob=-(centre_h-aruco_centre_y)*0.004065
			ref_vec=numpy.array((0,1))
			vec=numpy.array(([-(topRight[0]-bottomRight[0])],[-(topRight[1]-bottomRight[1])]))
			direction=1
			if (topRight[0]-bottomRight[0])!=0:
			  direction=(topRight[0]-bottomRight[0])/abs((topRight[0]-bottomRight[0]))
			angle=math.degrees(math.acos((numpy.dot(ref_vec,vec))/(numpy.linalg.norm(ref_vec)*numpy.linalg.norm(vec))))*direction

			#print(X_Rob,Y_Rob,angle)

			cv2.imshow("correction",	image)
			cv2.waitKey(3)		
			finish=1
	elif finish==0:
		Y_Rob=0
		X_Rob=0
		angle=0
		finish=2

# Main
rospy.init_node("detect_aruco_move")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
bridge = cv_bridge.CvBridge()
image_sub = rospy.Subscriber('/camera/rgb/image_raw',Image, image_callback)

speed = Twist()
r = rospy.Rate(4)

start_time=0
while not rospy.is_shutdown():
	#Checking marker detection success status and initial robot pose saved status
	if init==0 and finish==1:
		if first_run==0:
			# One time value assignment
			start_time = rospy.get_time()
			goal = Point()
			goal.x = X_Rob
			goal.y = Y_Rob
			goal_angle=angle
			max_lin_vel=0.2
			max_ang_vel=0.2
			first_run=1
			print("Robot Pose To marker",X_Rob,Y_Rob,angle)
		inc_x = goal.x -x
		inc_y = goal.y -y

		angle_to_goal = math.degrees(atan2(inc_y, inc_x))
		distance=math.sqrt((inc_x)**2+(inc_y)**2)

		if abs(angle_to_goal - theta) > 5 and (distance)>0.012:
			#print("heading angle",theta,abs(goal.x-x),abs(goal.y-y),distance)
			direction=1 if  distance<0.06 else ((angle_to_goal - theta)/abs(angle_to_goal - theta)) 
			speed.linear.x = 0.0
			speed.angular.z =(max_ang_vel if (math.radians(abs(angle_to_goal - theta))*0.7)>max_ang_vel else math.radians(abs(angle_to_goal - theta))*0.7)*direction
		elif distance>0.01:
			#print("heading straight",theta,abs(goal.x-x),abs(goal.y-y),distance)
			speed.linear.x = max_lin_vel if (0.5*distance)>max_lin_vel else 0.5*distance
			speed.angular.z = 0.0
		elif distance<0.01 and abs(goal_angle-theta) > 1:
			#print("correcting angle",theta,abs(goal.x-x),abs(goal.y-y),distance)
			direction=1 if goal_angle - theta==0 else ((goal_angle - theta)/abs(goal_angle - theta))
			speed.linear.x = 0.0
			speed.angular.z =(max_ang_vel if (math.radians(abs(goal_angle - theta))*0.7)>max_ang_vel else math.radians(abs(goal_angle - theta))*0.7)#*direction
		elif distance<0.01 and abs(goal_angle-theta) < 1:
			print("Robot Error ",abs(goal_angle-theta),abs(goal.x-x),abs(goal.y-y),distance)
			speed.linear.x = 0.0
			speed.angular.z = 0.0
			pub.publish(speed)
			stop_time = rospy.get_time() - start_time
			print("Robot Running time: ",stop_time)
			print("Goal Reached, Shutting down")
			break
	elif finish==2:
		rospy.logerr("Unable to detect marker")
		print("Shutting down")
		speed.linear.x = 0.0
		speed.angular.z = 0.0
		pub.publish(speed)
		break
	else:
		speed.linear.x = 0.0
		speed.angular.z = 0.0


	pub.publish(speed)
	r.sleep()    


