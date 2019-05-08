#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import time

'''
Created on 25-Apr-2019

@author: Davis
'''

class VisionTrackState(EventState):
    '''
    Threshold a HSV range

    --- lowerHSV 		list		Lower bound HSV values.
    --- upperHSV 		list		upper bound HSV values.
    ># img 					image from camera (uncompresed).

    #> value 					Value of float.

    <= done 					Published message.
    <= invalid 					No target in image.

    '''

    def __init__(self, lowerHSV=[0, 200, 0], upperHSV=[1, 255, 255]):
        '''
        Constructor
        '''
        super(VisionTrackState, self).__init__(outcomes=['done', 'invalid'], input_keys=['img'], output_keys=['value'])
        
        # sensor_msgs/Image
        # /multisense/camera/left/image_color
        # self._topic = topic

        self._min = [0, 200, 0]
        self._max = [1, 255, 255]

    def on_enter(self, userdata):
        self._img = userdata.img 
        
    def execute(self, userdata):
        print(type(userdata.img))
        np_arr = np.fromstring(userdata.img.data, np.uint8)
        orig = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #orig = CvBridge().imgmsg_to_cv2(userdata.img, "bgr8")
        #orig = np.fromstring(userdata.img, np.uint8)

        #make array for final values
        HSVLOW=np.array(self._min)
        HSVHIGH=np.array(self._max)

        obj = []
        
        Gaussianframe=cv2.GaussianBlur(orig,(5,5),0)
        
        #convert to HSV from BGR
        hsv=cv2.cvtColor(Gaussianframe, cv2.COLOR_BGR2HSV)
        
        #apply the range on a mask
        mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
        
        mask = cv2.dilate(mask, None, iterations=3)
        
        res = cv2.bitwise_and(Gaussianframe,Gaussianframe, mask =mask)
        
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        center = None
        
        for c in cnts:
            if len(c) > 3:
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                
                M = cv2.moments(c)
                if not(M["m00"] == 0):
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    #center.x = int(M["m10"] / M["m00"])
                    #center.y = int(M["m01"] / M["m00"])
                if cv2.contourArea(c) > 20 and radius > 60:
                    obj.append((x,y))
                    cv2.circle(Gaussianframe, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                    cv2.circle(Gaussianframe, center, 1, (0, 0, 255), -1)
        
        object_count = len(obj)
        # cv2.imshow("res img", res)

        height, width, channels = hsv.shape
        print("CENTER OF BALL: ", center)
        print("WIDTH x HEIGHT", width, height)
        print("obj count: ", object_count)

        if center != None:
            userdata.value = center[0]
            return "done"

        userdata.value = None
        return "invalid"