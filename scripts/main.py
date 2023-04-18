#! /usr/bin/python3

import rospy

import message_filters
from sensor_msgs.msg import Image

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv


def potholes_mask(rgb):
    # add paramd
    rgb = cv.flip(rgb, 0)

    # grayscale = cv.cvtColor(rgb, cv.COLOR_BGR2GRAY)

    # Color filtering
    hsv = cv.cvtColor(rgb.copy(), cv.COLOR_BGR2HSV)

    # Threshold of blue in HSV space
    lower_white = np.array([255, 255, 255])
    upper_white = np.array([200, 200, 200])

    # preparing the mask to overlay
    mask = cv.inRange(hsv, lower_white, upper_white)

    # The black region in the mask has the value of 0,
    # so when multiplied with original image removes all non-blue regions

    result = cv.bitwise_and(rgb, rgb, mask = mask)

    #cv.imshow('result', rgb)
    # Canny edge detection
    # edge = cv.Canny(result, 50 , 150)

    return rgb


def image_callback(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    final = potholes_mask(cv_image)

    try:
        pub.publish(bridge.cv2_to_imgmsg(final, encoding="passthrough"))
    except CvBridgeError as e:
        rospy.loginfo("no pub")
        print(e)


def start_node():
    rospy.init_node('potholes_mask')
    rospy.loginfo('pothole detcrtion started')
    global sub, pub
    sub = rospy.Subscriber(
        "/zed2i/zed_node/rgb/image_rect_color", Image, image_callback)
    pub = rospy.Publisher("/mask_pothole", Image, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
