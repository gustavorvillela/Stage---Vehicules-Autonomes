#!/usr/bin/env python3

from __future__ import division, print_function

import rospy

import cv2
import sys
import numpy as np

from sensor_msgs.msg import Image,CompressedImage


class Imager:

    def __init__(self):

        # Initializing publisher and subscriber
        self.image_pub = rospy.Publisher('filtered/compressed',CompressedImage,queue_size=10)
        self.camera_sub = rospy.Subscriber("raspicam_node/image/compressed",CompressedImage,self.im_callback)


    # Image callback exemple 
    def im_callback(self,im):

        '''Callback function of subscribed topic. 
        Here images get converted and edges detected'''
    
    #### direct conversion to CV2 ####
        np_arr = np.frombuffer(im.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        kernel = np.ones((5, 5), np.uint8)

    # convert np image to grayscale
        gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)


        #gray = cv2.erode(gray, kernel, iterations=1)
        #gray = cv2.dilate(gray, kernel, iterations=1)

        gray = cv2.Canny(gray, 50, 150, apertureSize=3)

    
    #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', gray)[1]).tobytes()
        # Publish new image
        self.image_pub.publish(msg)

if __name__ == "__main__":
    try:
        rospy.init_node('image_proc', anonymous=True)
        imag = Imager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass