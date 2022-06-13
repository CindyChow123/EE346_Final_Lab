#!/usr/bin/env python
# nav
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# follow
import cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
# from visualization_msgs.msg import Marker
import cv2.aruco
import time
import tf


# [-1.2, 3.8]

# x: 0.719473667978
# y: 2.4797341004
# z: 0.0
# orientation: 
# x: 0.0
# y: 0.0
# z: 0.0716256950306
# w: 0.997431581519
# nav
# Entry = [-2.32728338242,2.2037774086,0.186409196676,0.9824721937]
Entry = [-2.32728338242,2.15037774086,0.186409196676,0.9824721937]
Track1 = [-1.68295645714,2.43262887001,0.186403750602,0.982473226995]
Track2 = [-1.21254944801,1.86551582813,-0.562906524917,0.826520564902]
# P2 = [0.841003477573,2.49115228653,0.186401613163,0.982473632527]
P2 = [0.954940795898,2.61122846603,0.186401613163,0.982473632527]
Track3 = [0.06332051754,4.2484254837,0.844795774692,0.535088870248]
End1 = [-1.4708313942, 3.85947990417,0.982473010154,-0.186404893496]
End1front = [-1.808313942, 3.85947990417,-0.982473010154,0.186404893496]
Mid3 = [-2.27900242805,5.4757900238,0.826520757267,0.562906242466]
# P3 = [-0.573383712769,6.36572742462,0.186398304522,0.982474260259]
P3 = [-0.573383712769,6.36572742462,0.982474260259,-0.186398304522]
# Mid4 = [-2.53212332726,5.70930719376,0.982473299028,-0.186403370937]
# Mid4 = [-2.53212332726,5.70930719376,0.186398304522,0.982474260259]
Mid4 = [-2.75039601326,5.52666759491,0.982473987873,-0.186399740217]
# P4 = [-4.49689443588,4.70948839188,0.186389725471,0.186389725471]
P4 = [-4.54914474487,4.63593673706,0.186389725471,0.186389725471]
Back1 = [-2.49766802788,4.88461685181,-0.0149439253923,0.999888333312]
Back2 = [-1.73156607151,3.04755735397,-0.562906574181,0.82652053135]
# Back3 = [-1.47458207607,2.50390386581, 0.982473832328,-0.186400560058]
# P1 = [-3.05020236969, 0.991712510586,0.116284845178,0.993215905421]
# Back3 = [-1.47458207607,2.36390386581, 0.982473832328,-0.186400560058]
Back3 = [-1.47458207607,2.36390386581,0.982473832328,-0.186400560058]
# Back3 = [-1.54458207607,2.36390386581,0.982473832328,-0.186400560058]
# P1 = [-3.09247136116, 1.07179522514,0.826521059233,0.562905799086]
# P1 = [-3.09247136116, 1.07179522514,0.116284845178,0.993215905421]
# P1 = [-3.14067983627,1.08505964279,0.826520589508,0.562906488788]
P1 = [-3.14067983627,0.88505964279,0.826520589508,0.562906488788]

# Nav Track Nav
# track_end = [Entry,End1,Mid3,P3,P4,Back1,Back3,P1]
# track_end = [Entry,Mid3,P3,P4,Back1,Back3,P1]
# track_end = [Entry,End1front,Back3,P1]
# Pure Navigation
track_end = [Entry,Track1,Track2,P2,Track3,End1,Mid3,P3,Mid4,P4,Back1,Back3,P1]
# track_end = [Entry,End1,Back3,P1]

# follow
wall_threshold = 10000000
turn_delay = 0.6 #0.8
wall_delay = 1
turn_vel_x = 0.09 #0.12
max_vel_x = 0.2
max_turn_vel = 0.035
pub_flag = True
straight_delay = 0.85


def end_detection(bin_image):
    bin_image = cv2.medianBlur(bin_image,15)
    h, w = bin_image.shape
    back_img = numpy.zeros((h,w))
    back_img.fill(0)
    canny_img = cv2.Canny(bin_image, 30, 150) #30 150
    lines = cv2.HoughLines(canny_img, 1, numpy.pi / 180, 60) #90 

    trans_count = 0

    if lines is not None:
        lines1 = lines[:, 0, :]

        for rho, theta in lines1[:]:
            # a = numpy.cos(theta)
            # b = numpy.sin(theta)
            # x0 = a * rho
            # y0 = b * rho
            # x1 = int(x0 + 3000 * (-b))
            # y1 = int(y0 + 3000 * (a))
            # x2 = int(x0 - 3000 * (-b))
            # y2 = int(y0 - 3000 * (a))
            # cv2.line(back_img, (x1, y1), (x2, y2), 255, 2)
            k = numpy.arctan(numpy.cos(theta)/(numpy.sin(theta)))
            if abs(k) < 0.2:
                trans_count = trans_count + 1

    if trans_count:
        rospy.loginfo("transverse line detected!")
        return 1
    else:
        return 0
        


def start_detection(bin_image):
    bin_image = cv2.medianBlur(bin_image,15)
    canny_img = cv2.Canny(bin_image, 30, 150) #30 150
    lines = cv2.HoughLines(canny_img, 1, numpy.pi / 180, 60) #90 

    vert_count = 0

    if lines is not None:
        lines1 = lines[:, 0, :]

        for rho, theta in lines1[:]:
            k = numpy.arctan(numpy.cos(theta)/(numpy.sin(theta)))
            if abs(k) > 1:
                vert_count = vert_count + 1

    if not vert_count:
        rospy.loginfo("no any vertical lines detected!")
        return 1
    else:
        return 0

class nav_client():

	def __init__(self):
		self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.goal = MoveBaseGoal
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

		self.listener = tf.TransformListener() 

		while not self.ac.wait_for_server(rospy.Duration(5.0)):
			rospy.loginfo("Waiting for the move_base action server to come up")

		self.goal = MoveBaseGoal()

		rospy.loginfo('Map')
		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.header.stamp = rospy.Time.now()
		self.navigating = True
		self.max_rounds = 5
		self.current_round = 1

		# Follow
		self.bridge = cv_bridge.CvBridge()
                
		self.image_sub = rospy.Subscriber('/usb_cam/image_raw',
				Image, self.image_callback, queue_size=1)

		# rospy.loginfo(self.image_sub.queue_size)

		self.cmd_vel_pub = rospy.Publisher('cmd_vel',
				Twist, queue_size=10)

		self.twist = Twist()

		# self.K = numpy.mat([[265, 0, 160], [0, 265, 120], [0, 0, 1]])
		self.K = numpy.mat([[502.51216,   0.     , 322.67149],
							[0.     , 502.86933, 226.71022],
							[0.     ,   0.     ,   1.   ]])
		self.distCoeffs = numpy.ndarray([0])
		self.marker_len = 10
		t = numpy.mat([0, -0.5, 0.7]).T
		n = numpy.mat([0, -1, 0])
		R = numpy.mat([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
		d = 0.115

		self.H = self.K*(R-t*n/d)*numpy.linalg.inv(self.K)

		self.last_err = 0
		self.d_count = False

		self.last_cx_left = 0
		self.last_cx_right = 0

		self.last_ang_z = 0
		self.last_vel_x = 0

		self.turn_flag = False
		self.turn_count = 0
		self.status_count = 0
		self.sleep_flag = False
		self.sleep_t = 0
		self.sleep_count = 0
		self.start_count = 115
		self.start_flag = True
		self.start_status1_flag = False

		self.turn_stamp = 0
		self.end_stamp = 0

		self.end_flag = False
		self.end_line_flag = False

		self.finish_flag = False
		self.trans = None

		while self.current_round < self.max_rounds:

			# Phase 1: Navigate to the track
			# self.navigate(True)
			# self.navigating = True
			# Phase 2: Follow the track
			# while not self.navigating:
			# 	continue
			# Phase 3: Navigate Point 3 and 4
			# self.turn_count = 0
			# self.navigate = True
			self.navigate(False)
			self.current_round = self.current_round + 1
			
	
	def navigate(self,first):
		if first:
			self.move(track_end[0])
			# self.shut()
		else:
			for i in range(0,len(track_end)):
				self.move(track_end[i])
			# rospy.on_shutdown(self.shut())

	def move(self,point):
		self.goal.target_pose.pose.position.x = point[0]
		self.goal.target_pose.pose.position.y = point[1]
		self.goal.target_pose.pose.orientation.z = point[2]
		self.goal.target_pose.pose.orientation.w = point[3]

		rospy.loginfo('Sending goal')
		self.ac.send_goal(self.goal)

		finished_within_time = self.ac.wait_for_result(rospy.Duration(30))
		rospy.loginfo(finished_within_time)

		if finished_within_time:
			rospy.loginfo('Success!')
		else:
			rospy.loginfo('Failed!')

	def shut(self):
		rospy.loginfo("Stopping the robot...")
		self.ac.cancel_goal()
		rospy.sleep(2)
		self.cmd_vel_pub.publish(Twist())
		rospy.sleep(1)

	def init_params(self):
		self.turn_count = 0
		self.navigating = True

	def image_callback(self, msg):

		# rospy.loginfo(self.navigating)
		# if self.navigating:
		# 	self.navigate(True)
		# 	self.navigating = False
		# 	return
		# elif self.turn_count == 3:
		# 	self.navigate(False)
		# 	self.init_params()
		# 	return

		if self.navigating:
			return
		
		try:
			(self.trans, rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
			if self.trans[0] < -1.4 and self.trans[1] > 3.8:
				self.navigating = True
				rospy.loginfo('Nav Begin')
				return
		except:
			rospy.loginfo('ERROR')

        # try:
		# 	(trans,rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0)) 
        # except Exception as r:
        # 	rospy.loginfo(r)
		
			# return


		if self.finish_flag:
			self.twist.linear.x = 0
			self.twist.angular.z = 0
			if pub_flag:
				self.cmd_vel_pub.publish(self.twist)
			return
		current_t = time.time()

		if self.sleep_flag:
			if (current_t-self.sleep_t) < self.sleep_count:
				return
			else:
				self.sleep_flag = False


		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		h, w, d = image.shape
		bev_color = cv2.warpPerspective(image, self.H, (w,h))
		bev_color = cv2.flip(bev_color, 0)

		# used to find the line
		lower_black = numpy.array([0, 0, 0])
		upper_black = numpy.array([255, 100, 87])

		# used to find the wall
		lower_wall = numpy.array([0, 0, 70])
		upper_wall = numpy.array([110, 255, 130])

		# lower_wall = numpy.array([0, 0, 70])
		# upper_wall = numpy.array([255, 255, 127])
		# upper_black = numpy.array([180, 255, 90])

		# fliter first, then homo
		mask1 = cv2.inRange(image, lower_black, upper_black)
		mask1 = cv2.warpPerspective(mask1, self.H, (w,h))
		mask1 = cv2.flip(mask1, 0)
		# mask1 = cv2.medianBlur(mask1,15)
		# mask1 = cv2.inRange(hsv, lower_black, upper_black)

		# fliter first, then homo
		mask_wall = cv2.inRange(image, lower_wall, upper_wall)
		mask_wall = cv2.warpPerspective(mask_wall, self.H, (w,h))
		mask_wall = cv2.flip(mask_wall, 0)

		# start stage   
		if self.start_flag:

			if self.start_status1_flag:
				self.start_flag = False
				rospy.loginfo("turn right--start stage")
				# turn right
				self.twist.linear.x = 0
				self.twist.angular.z = -0.5
				if pub_flag:
					self.cmd_vel_pub.publish(self.twist)
				# delay
				# self.last_ang_z = self.twist.angular.z
				# self.last_vel_x = self.twist.linear.x
				self.sleep_flag = True
				self.sleep_t = current_t
				self.sleep_count = 1.6
				return

			if start_detection(mask1):
				self.start_status1_flag = True
				rospy.loginfo("straight delay")
				self.twist.linear.x = 0.3
				self.twist.angular.z = 0
				if pub_flag:
					self.cmd_vel_pub.publish(self.twist)
				# delay
				# self.last_ang_z = self.twist.angular.z
				# self.last_vel_x = self.twist.linear.x
				self.sleep_flag = True
				self.sleep_t = current_t
				self.sleep_count = straight_delay

			else:
				# go straight
				row_center = h/2
				row_len = h/6

				M2 = cv2.moments(mask1[row_center-row_len:row_center+row_len, :])
						
				cx2 = int(M2['m10']/M2['m00'])
				cy2 = int(M2['m01']/M2['m00'])
				err = w/2 - cx2
				self.twist.angular.z = (err*90.0/160)/15*max_turn_vel
				self.twist.linear.x = 0.2
				# self.twist.angular.z = 0
				if pub_flag:
					self.cmd_vel_pub.publish(self.twist)

			return


		# end stage
		if self.turn_count == 100 and (current_t-self.end_stamp)>8 and not self.end_flag:
		# if (current_t-start_t)>33 and (current_t-self.end_stamp)>8 and not self.end_flag:
			if end_detection(mask1):
				self.end_flag = True

				self.twist.linear.x = 0.3
				self.twist.angular.z = 0
				if pub_flag:
					self.cmd_vel_pub.publish(self.twist)
				# delay
				self.sleep_flag = True
				self.sleep_t = current_t
				self.sleep_count = 1.7
				return


		if self.end_flag:
			# self.turn_count = 0
			# turn right
			self.end_flag = False
			self.end_line_flag = True
			self.twist.linear.x = 0
			self.twist.angular.z = -0.5
			if pub_flag:
				self.cmd_vel_pub.publish(self.twist)
			# delay
			# self.last_ang_z = self.twist.angular.z
			# self.last_vel_x = self.twist.linear.x
			self.sleep_flag = True
			self.sleep_t = current_t
			self.sleep_count = 1.2
			rospy.loginfo("end turn")
			return

		# if self.end_line_flag:
		#     self.finish_flag =  True
		#     self.end_line_flag = False

		#     self.twist.linear.x = 0.3
		#     self.twist.angular.z = 0
		#     if pub_flag:
		#         self.cmd_vel_pub.publish(self.twist)
		#     # delay
		#     self.sleep_flag = True
		#     self.sleep_t = current_t
		#     self.sleep_count = 1.7
		#     return



		# for debugging usage
		# cv2.line(bev_color, (10, h/3), (w-10, h/3), (0, 0, 255), 2)
		# cv2.line(bev_color, (10, h/5), (w-10, h/5), (0, 0, 255), 2)
		# cv2.line(bev_color, (10, h/2), (w-10, h/2), (0, 255, 0), 2)
		# cv2.line(bev_color, (10, h/6), (w-10, h/6), (0, 255, 0), 2)
		# cv2.line(bev_color, (10, 2*h/3), (w-10, 2*h/3), (0, 0, 255), 2)
		# cv2.line(bev_color, (10, 5*h/6), (w-10, 5*h/6), (0, 0, 255), 2)


		row_center = h/2
		row_len = h/6

		M1 = cv2.moments(mask1[0:h/3, :])
		M2 = cv2.moments(mask1[row_center-row_len:row_center+row_len, :])
		M_wall = cv2.moments(mask_wall[0:h/5, :])


		# wall detection
		# hard code threshold 
		if (M_wall['m00']) > wall_threshold:
			rospy.loginfo("wall")
			self.twist.linear.x = 0.05
			self.twist.angular.z = 0.3
			if pub_flag:
				self.cmd_vel_pub.publish(self.twist)

			# use the timer to sleep
			self.sleep_count = wall_delay
			self.sleep_flag = True
			self.sleep_t = current_t
			# self.last_ang_z = self.twist.angular.z
			# self.last_vel_x = self.twist.linear.x


		# close point
		elif (M2['m00']) > 0:

			cx2 = int(M2['m10']/M2['m00'])
			cy2 = int(M2['m01']/M2['m00'])

			cv2.circle(bev_color, (cx2, row_center), 10, (255, 0, 0), -1)

			err = w/2 - cx2

			# far point
			# used for turning
			if (M1['m00']) > 0:
				if self.turn_flag:
					self.turn_flag = False
					# rospy.loginfo("straight")
					if (current_t-self.turn_stamp) > 10:
						self.turn_count = self.turn_count + 1
						rospy.loginfo("turn"+str(self.turn_count))
						if self.turn_count == 3:
							self.end_stamp = current_t
						self.turn_stamp = current_t


					# used the timer to sleep for a while
					self.sleep_count = turn_delay
					self.sleep_flag = True
					self.sleep_t = current_t


				cx1 = int(M1['m10']/M1['m00'])
				cy1 = int(M1['m01']/M1['m00'])                    
				cv2.circle(bev_color, (cx1, h/6), 10, (0, 0, 255), -1)

				err_far = w/2 - cx1

				linear_x = max_vel_x-abs(err)*0.001
				self.twist.linear.x = linear_x if linear_x > 0.05 else 0.05

			# turn
			else:
				if not self.turn_flag:
					self.turn_flag = True

				# rospy.loginfo("turning........")
				self.twist.linear.x = turn_vel_x

			# use the close point for adjustment
			self.twist.angular.z = (err*90.0/160)/15*max_turn_vel
			self.last_ang_z = self.twist.angular.z
			# self.last_vel_x = self.twist.linear.x

			# pub the vel cmd
			if pub_flag:
				self.cmd_vel_pub.publish(self.twist)


		# when lose both close and far point, pure rotation
		else:
			self.twist.linear.x = 0
			self.twist.angular.z = self.last_ang_z
			if pub_flag:
				self.cmd_vel_pub.publish(self.twist)

		
		cv2.imshow("window", mask1)
		cv2.imshow("mask", mask_wall)

		cv2.waitKey(1)


if __name__ == '__main__':

	try:
		rospy.init_node('nav_follow_nav')
		c = nav_client()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("AMCL navigaton stopped!")
