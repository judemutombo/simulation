#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
class ProcessController :

    def __init__(self):
        
        self._subcamqr = rospy.Subscriber("camera_qr_code_feed",Image, self._camqrProcess)
        self._bridge = CvBridge()
        self._camqrFrame = None

    def _camqrProcess(self, data):
        try: 
            cv_image = self._bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            self._camqrFrame = cv_image
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")