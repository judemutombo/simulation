#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from task import Task
from ie_communication.msg import TaskData
from ie_communication.srv import  robotTask, robotTaskResponse
class ProcessController :

    def __init__(self):
        
        self._subcamqr = rospy.Subscriber("camera_qr_code_feed",Image, self._camqrProcess)
        self._service = rospy.Service('camera_state', robotTask, self.setTask)
        self._bridge = CvBridge()
        self._camqrFrame = None
        self._currentTask = None

    def _camqrProcess(self, data):
        try: 
            cv_image = self._bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            self._camqrFrame = cv_image
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
    
    def processing(self):
        pass
    
    def setTask(self, req):

        if self._currentTask is not None:
            if self._currentTask.running:
                return robotTaskResponse(message="a task is already running")
    
        self._currentTask = Task(req.task.task_name)
        return robotTaskResponse(message="task set")

if __name__ == '__main__':
    from threading import Thread

    rospy.init_node('ProcessController', disable_signals=True)
    pr = ProcessController()
    rospy.spin()