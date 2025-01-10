#!/usr/bin/env python3

import rospy
from ie_communication.srv import camState, camStateResponse
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraController:

    def __init__(self):
        self._pubcam1 = rospy.Publisher("camera_feed", Image, queue_size=10)
        self._pubcam2 = rospy.Publisher("camera_qr_code_feed", Image, queue_size=10)

        self._subcam1 = rospy.Subscriber("/camera/rgb/image_raw",Image, self._camera1Process)
        self._subcam2 = rospy.Subscriber("/camera_2/camera_2/image_raw",Image, self._camera2Process)

        self._service = rospy.Service('camera_state', camState, self._changeCameraState)
        self._bridge = CvBridge()
        self._camState = True
        self._cam_available = False
        self._cam1Frame = None
        self._cam2Frame = None
        print("cameraController initialized")

    def _camera1Process(self, data):
        try:
            # Convert the ROS Image message to an OpenCV-compatible format
            self._cam1Frame = data
            #cv_image = self._bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            #cv2.imshow("camera",cv_image)
            self._cam_available = True
            self._provideCamFeed()
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def _camera2Process(self, data):
        try:
            # Convert the ROS Image message to an OpenCV-compatible format
            self._cam2Frame = data
            #cv_image = self._bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            #cv2.imshow("camera",cv_image)
            self._cam_available = True
            self._provideCam2Feed()
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def _changeCameraState(self, req):
        self._camState = req.state
        print(f"Camera state changed to: {self._camState}")
        return camStateResponse(success=True)

    def _provideCamFeed(self) -> None:
        if self._camState and self._cam_available:
            try:
                self._pubcam1.publish(self._cam1Frame)
            except Exception as e:
                rospy.logerr(f"Error publishing image feed: {e}")
    
    def _provideCam2Feed(self) -> None:
        try:
            self._pubcam2.publish(self._cam2Frame)
        except Exception as e:
            rospy.logerr(f"Error publishing image from cam2 feed: {e}")
    
        
if __name__ == '__main__':
    from threading import Thread

    rospy.init_node('CameraController')
    cam = CameraController()
    rospy.spin()
