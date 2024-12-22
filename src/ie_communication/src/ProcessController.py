#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
from task import Task
from ie_communication.msg import TaskData
from ie_communication.srv import  robotTask, robotTaskResponse
class ProcessController :

    def __init__(self):
        
        self._subcamqr = rospy.Subscriber("/camera_qr_code_feed",Image, self._camqrProcess)
        self._service = rospy.Service('robot_task', robotTask, self.setTask)
        self._pubGear = rospy.Publisher('/robot_gear', Int32, queue_size=10)
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
        rospy.loginfo(f"Received task request: {req.task.task_name}")
        if self._currentTask is not None:
            if self._currentTask.running:
                return robotTaskResponse("a task is already running")
    
        self._currentTask = Task(req.task.task_name)
        self._currentTask.start()
        self._pubGear.publish(1)
        return robotTaskResponse("task set")

if __name__ == '__main__':
    from threading import Thread

    rospy.init_node('ProcessController')
    pr = ProcessController()
    rospy.spin()