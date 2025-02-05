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
        self._pubcam3 = rospy.Publisher("camera_left_feed", Image, queue_size=10)
        self._pubcam4 = rospy.Publisher("camera_right_feed", Image, queue_size=10)

        rospy.Subscriber("/camera/rgb/image_raw",Image, self._camera1Process)
        rospy.Subscriber("/camera_2/camera_2/image_raw",Image, self._camera2Process)
        rospy.Subscriber("/camera_left/camera_left/image_raw",Image, self._camera3Process)
        rospy.Subscriber("/camera_right/camera_right/image_raw",Image, self._camera4Process)

        self._bridge = CvBridge()
        self._camState = True
        self._cam_available = False
        self._cam1Frame = None
        self._cam2Frame = None
        self._cam3Frame = None
        self._cam4Frame = None
        # print("cameraController initialized")

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

    def _camera3Process(self, data):
        try:
            # Convert the ROS Image message to an OpenCV-compatible format
            self._cam3Frame = data
            #cv_image = self._bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            #cv2.imshow("camera",cv_image)
            self._cam_available = True
            self._provideCam3Feed()
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
    
    def _camera4Process(self, data):
        try:
            # Convert the ROS Image message to an OpenCV-compatible format
            self._cam4Frame = data
            #cv_image = self._bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            #cv2.imshow("camera",cv_image)
            self._cam_available = True
            self._provideCam4Feed()
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

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

    def _provideCam3Feed(self) -> None:
        try:
            self._pubcam3.publish(self._cam3Frame)
        except Exception as e:
            rospy.logerr(f"Error publishing image from cam3 feed: {e}")
    
    def _provideCam4Feed(self) -> None:
        try:
            self._pubcam4.publish(self._cam4Frame)
        except Exception as e:
            rospy.logerr(f"Error publishing image from cam4 feed: {e}")
    
        
if __name__ == '__main__':
    from threading import Thread

    rospy.init_node('CameraController')
    cam = CameraController()
    rospy.spin()
