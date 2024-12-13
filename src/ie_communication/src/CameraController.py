#!/usr/bin/env python3

import rospy
from std_msgs.msg import Byte
from ie_communication.srv import camState, camStateResponse
import cv2
import time
from threading import Lock
from cv_bridge import CvBridge, CvBridgeError

class CameraController:

    def __init__(self):
        self._pub = rospy.Publisher("camera_feed", Byte, queue_size=10)
        self._service = rospy.Service('camera_state', camState, self._changeCameraState)
        self._camState = True
        self._camera_disconnected = False
        self._cam_available = False
        self._lock = Lock()
        self._cam = None
        print("cameraController initialized")

    def cameraProcess(self):
        self._cam = cv2.VideoCapture(0)
        self._cam .set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._cam .set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        while not rospy.is_shutdown():
            with self._lock:
                if not self._cam.isOpened():
                    if not self._camera_disconnected:
                        print("Camera not found. Retrying...")
                        self._camera_disconnected = True
                        self._cam_available = False
                    time.sleep(1)
                    self._cam = cv2.VideoCapture(0)
                    continue

                self._camera_disconnected = False
                self._cam_available = True

            ret, frame = self._cam.read()
            if not ret:
                with self._lock:
                    if not self._camera_disconnected:
                        print("Failed to read from camera.")
                        self._camera_disconnected = True
                        self._cam_available = False
                time.sleep(1)
                continue
            print("capturing frame")
            self._camFrame = frame
            self._provideCamFeed()

    def _changeCameraState(self, req):
        self._camState = req.state
        print(f"Camera state changed to: {self._camState}")
        return camStateResponse(success=True)

    def _provideCamFeed(self) -> None:
        print("Publishing camera feed...")
        if self._camState and self._cam_available:
            ret, buffer = cv2.imencode('.png', self._camFrame)
            if ret:
                self._pub.publish(buffer.tobytes())
            else:
                print("Error encoding frame to PNG.")
        
if __name__ == '__main__':
    from threading import Thread

    rospy.init_node('camera_node', disable_signals=True)
    cam = CameraController()
    
    # Run camera process in a separate thread
    camera_thread = Thread(target=cam.cameraProcess)
    camera_thread.start()
    
    rospy.spin()
    camera_thread.join()
