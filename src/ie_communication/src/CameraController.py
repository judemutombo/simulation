#!/usr/bin/env python3

import rospy
from ie_communication.srv import camState, camStateResponse
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

class CameraController:

    def __init__(self):
        self._pub = rospy.Publisher("camera_feed", Image, queue_size=10)
        self._sub = rospy.Subscriber("/camera/rgb/image_raw",Image, self._cameraProcess)
        self._service = rospy.Service('camera_state', camState, self._changeCameraState)
        self._bridge = CvBridge()
        self._camState = True
        self._cam_available = False
        self._camFrame = None
        print("cameraController initialized")

    def _cameraProcess(self, data):
        try:
            # Convert the ROS Image message to an OpenCV-compatible format
            self._camFrame = data
            self._cam_available = True
            self._provideCamFeed()
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")


    def _changeCameraState(self, req):
        self._camState = req.state
        print(f"Camera state changed to: {self._camState}")
        return camStateResponse(success=True)

    def _provideCamFeed(self) -> None:
        if self._camState and self._cam_available:
            try:
                self._pub.publish(self._camFrame)
            except Exception as e:
                rospy.logerr(f"Error publishing image feed: {e}")
    
        
if __name__ == '__main__':
    from threading import Thread

    rospy.init_node('CameraController', disable_signals=True)
    cam = CameraController()
    rospy.spin()
