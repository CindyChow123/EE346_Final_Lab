#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
# from visualization_msgs.msg import Marker
import cv2.aruco


# pre_mask  = 5346000
# pre_mask_10 = 1695100000
# pre_mask_01 = 2216200000


pre_mask =  20124000
pre_mask_10 = 6392000000
pre_mask_01 = 7287000000


class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('/usb_cam/image_raw',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()

                # self.K = numpy.mat([[265, 0, 160], [0, 265, 120], [0, 0, 1]])
                self.K = numpy.mat([[502.51216,   0.     , 322.67149],
                                    [0.     , 502.86933, 226.71022],
                                    [0.     ,   0.     ,   1.   ]])
                self.distCoeffs = numpy.ndarray([0])
                self.marker_len = 10
                t = numpy.mat([0, -0.59, 0.9]).T
                n = numpy.mat([0, -1, 0])
                R = numpy.mat([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
                d = 0.115

                self.H = self.K*(R-t*n/d)*numpy.linalg.inv(self.K)

                self.last_err = 0
                self.d_count = False





        def two_moment_compute(self, bin_image, start, row_len):

                h, w = bin_image.shape
                split = 2*w/5
                M_left = cv2.moments(bin_image[start:start+row_len, 0:split])
                M_right = cv2.moments(bin_image[start:start+row_len, w-split:])


                if M_left['m00'] == 0:
                    x_left = self.last_cx_left
                else:
                    x_left = int(M_left['m10']/M_left['m00'])
                
                if M_right['m00'] == 0:
                    x_right = self.last_cx_right
                else:
                    x_right = int(M_right['m10']/M_right['m00'])+w-split

                self.last_cx_left = x_left
                self.last_cx_right= x_right

                return x_left, x_right


        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                h, w, d = image.shape

                bev_image = cv2.warpPerspective(image, self.H, (w,h))
                bev_image = cv2.flip(bev_image, 0)
                hsv = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
                gray = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
                threshold, bin_gray = cv2.threshold(gray, 127, 255, cv2.THRESH_OTSU | cv2.THRESH_BINARY_INV)
                # threshold, bin_gray = cv2.threshold(gray, 127, 255, cv2.THRESH_OTSU | cv2.THRESH_BINARY)
                canny_img = cv2.Canny(bin_gray, 30, 150)

                # lines = cv2.HoughLines(canny_img, 1, numpy.pi / 180, 150)
                # lines1 = lines[:, 0, :]
                # for rho, theta in lines1[:]:
                #         a = numpy.cos(theta)
                #         b = numpy.sin(theta)
                #         x0 = a * rho
                #         y0 = b * rho
                #         x1 = int(x0 + 3000 * (-b))
                #         y1 = int(y0 + 3000 * (a))
                #         x2 = int(x0 - 3000 * (-b))
                #         y2 = int(y0 - 3000 * (a))
                #         cv2.line(bev_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

                
                lower_black = numpy.array([100, 0, 0])
                upper_black = numpy.array([255, 100, 120])

                # mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask1 = cv2.inRange(hsv, lower_black, upper_black)

                # h, w, d = bev_image.shape
                search_top = 2*h/3
                search_bottom = 1*h/3
                # mask1[0:search_top, 0:w] = 0
                # mask2[0:search_top, 0:w] = 0
                # mask1[search_bottom:h, 0:w] = 0
                # mask2[search_bottom:h, 0:w] = 0

                # bin_gray[0:search_top, 0:w] = 0

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(bin_gray)

                M_canny = cv2.moments(canny_img)
                # M2 = cv2.moments(mask2)


                if (M2['m00']) > 0:

                    cx1 = int((M2['m10']-pre_mask_10)/(M2['m00']-pre_mask))
                    cy1 = int((M2['m01']-pre_mask_01)/(M2['m00']-pre_mask))

                    # cx1 = int(M2['m10']/M2['m00'])
                    # cy1 = int(M2['m01']/M2['m00'])


                    # cx2 = int(M_canny['m10']/M_canny['m00'])
                    # cy2 = int(M_canny['m01']/M_canny['m00'])

                #     fpt_x = (cx1 + cx2)/2
                #     fpt_y = (cy1 + cy2)/2 + 2*h/3
                    fpt_x = cx1
                    fpt_y = cy1 + 2*h/3

                    cv2.circle(image, (cx1, cy1), 10, (0,255,255), -1)
                #     cv2.circle(image, (cx2, cy2), 10, (255,255,255), -1)
                #     cv2.circle(image, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    cv2.circle(bin_gray, (cx1, cy1), 10, (255,255,255), -1)
                    # cv2.circle(canny_img, (cx2, cy2), 10, (0,255,255), -1)
                #     cv2.circle(bev_image, (cx2, cy2), 10, (255,255,255), -1)
                #     cv2.circle(bev_image, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x
                    err_y = 5*h/6 - cy1
                #     err = w/2 - cx2

                    linear_x = 0.4-abs(err)*0.01 
                    self.twist.linear.x = linear_x if linear_x > 0.1 else 0.1
                    self.twist.angular.z = (err*90.0/160)/15*0.4
                    if self.d_count:
                        self.twist.angular.z = self.twist.angular.z + 0.0008*(err-self.last_err)*90.0/160

                    
                    self.cmd_vel_pub.publish(self.twist)
                    self.last_err = err
                #     self.d_count = True
                    # rospy.loginfo(self.twist.angular.z)
                    rospy.loginfo(M2['m01'])
                    # rospy.loginfo(err_y)

                    

                
                cv2.imshow("window", image)
                cv2.imshow("BEV", bev_image)
                cv2.imshow("BIN", bin_gray)
                # cv2.imshow("mask", mask1)
                # cv2.imshow("gray", bin_gray)
                # cv2.imshow("canny", canny_img)
                # cv2.imshow("mask", mask1+mask2)
                cv2.waitKey(1)

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
