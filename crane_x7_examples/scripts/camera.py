#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import UInt8
import cv2
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge

def process_image(msg):
    try:
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
        cv2.imshow('image', img)
        cv2.waitKey(1)
    except Exception as err:
        print err



def detect_red_color(img):
   
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_min = np.array([0,64,0])                # 赤色のHSVの値域1
    hsv_max = np.array([30,255,255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    
    hsv_min = np.array([150,64,0])              # 赤色のHSVの値域2
    hsv_max = np.array([179,255,255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

        
    mask = mask1 + mask2                       # 赤色領域のマスク（255：赤色、0：赤色以外）

   
    masked_img = cv2.bitwise_and(img, img, mask=mask)            # マスキング処理

    return mask, masked_img

    red_mask, red_masked_img = detect_red_color(img)

    cv2.imwrite("C:\prog\python\\test\\red_mask.png", red_mask)
    cv2.imwrite("C:\prog\python\\test\\red_masked_img.png", red_masked_img)

def start_node():
    rospy.init_node('img_proc')
    rospy.loginfo('img_proc node started')
    rospy.Subscriber("image_raw", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
