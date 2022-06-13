#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
# from visualization_msgs.msg import Marker
import cv2.aruco
import time
import tf


# param
wall_threshold = 10000000
turn_delay = 0.8 #0.8
wall_delay = 1
turn_vel_x = 0.1 #0.12
max_vel_x = 0.4
max_turn_vel = 0.045

pub_flag = True
#==============================
# todo:
# complete the programe of start and end stage
# test the delay time for both stage
# aruco marker detection part
#==============================

# for start stage: detect vertical lines, if no, turn right
# for end stage: detect transverse lines, if any, turn right
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

    # cv2.putText(back_img, str(k), (5,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 255, 2)
    # cv2.imshow("fork", back_img)
    # cv2.imshow("canny", canny_img)

    if trans_count:
        rospy.loginfo("transverse line detected!")
        return 1
    else:
        return 0
            # a = numpy.cos(theta)
            # b = numpy.sin(theta)
            # x0 = a * rho
            # y0 = b * rho
            # x1 = int(x0 + 3000 * (-b))
            # y1 = int(y0 + 3000 * (a))
            # x2 = int(x0 - 3000 * (-b))
            # y2 = int(y0 - 3000 * (a))
            # cv2.line(back_img, (x1, y1), (x2, y2), 255, 2)



        # cv2.putText(back_img, str(numpy.arctan(numpy.cos(theta)/(numpy.sin(theta)))), (5,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 255, 2)
        # cv2.putText(bev_color, str(theta), (5,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(255,0,0), 0, 2)

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



class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()
                
                self.image_sub = rospy.Subscriber('/usb_cam/image_raw',
                        Image, self.image_callback, queue_size=1)

                # rospy.loginfo(self.image_sub.queue_size)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()

                self.listener = tf.TransformListener() 

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

                self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
                self.arucoParams = cv2.aruco.DetectorParameters_create()
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
        

        def image_callback(self, msg):

                if self.finish_flag:
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0
                    if pub_flag:
                        self.cmd_vel_pub.publish(self.twist)
                    return
                current_t = time.time()
                rospy.loginfo(str(current_t-start_t))

                

                # straight and turn
                # hard code
                # if (current_t-start_t) < 3.6:        
                #     self.twist.linear.x = 0.3
                #     self.twist.angular.z = 0
                #     self.cmd_vel_pub.publish(self.twist)

                #     rospy.loginfo(current_t-start_t)
                #     return
                # elif (current_t-start_t) < 5.3:
                #     self.twist.linear.x = 0
                #     self.twist.angular.z = -0.5
                #     self.cmd_vel_pub.publish(self.twist)
                #     return

                # used to delay (turn and wall-turn)
                if self.sleep_flag:
                    if (current_t-self.sleep_t) < self.sleep_count:
                        # self.twist.linear.x = self.last_vel_x
                        # self.twist.angular.z = self.last_ang_z
                        # self.cmd_vel_pub.publish(self.twist)
                        return
                    else:
                        self.sleep_flag = False
                        # rospy.loginfo("finish")

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
                        self.sleep_count = 0.9

                    else:
                        # go straight
                        self.twist.linear.x = 0.3
                        self.twist.angular.z = 0
                        if pub_flag:
                            self.cmd_vel_pub.publish(self.twist)

                    return


                # end stage

                try:
                    (self.trans, rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
                    if self.trans[0] < -1.4 and self.trans[1] > 3.8 and (current_t-start_t)>90 and not self.end_flag:
                        if end_detection(mask1):
                        # if end_detection(mask1):
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

                except:
                    rospy.loginfo('ERROR')      


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

                    self.end_line_flag = True

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

                # end_detection(mask1)


                # corners = cv2.goodFeaturesToTrack(back_img, 4, 0.9, 20)
                # # try:
                # if corners is not None:
                #     for i in corners:
                #         x, y = i.ravel()
                #         cv2.circle(back_img, (int(x), int(y)), 10, 200, -1)
                # # except AttributeError:


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

                
                if self.end_line_flag:
                    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, self.arucoDict, parameters=self.arucoParams)

                    if len(corners) > 0:
                            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_len, self.K, self.distCoeffs)
                            rospy.loginfo("There is a marker! Distance: %f" % tvec[0][0][2])
                            image = cv2.aruco.drawDetectedMarkers(image, corners, ids)
                            if tvec[0][0][2] <= 35:
                                    self.twist.linear.x = 0
                                    self.twist.angular.z = 0
                                    self.cmd_vel_pub.publish(self.twist)
                                    rospy.loginfo("STOP HERE!!!!!!!!!!!!!!")
                                    rospy.sleep(10)
                       


                cv2.imshow("window", image)
                cv2.imshow("mask", mask1)

                cv2.waitKey(1)
                                


start_t = time.time()
rospy.init_node('lane_follower')
follower = Follower()
rospy.loginfo("init")
rospy.spin()
