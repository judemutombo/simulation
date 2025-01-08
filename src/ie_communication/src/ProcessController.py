#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
from task import Task
from mapping import Mapping
from ie_communication.msg import TaskData
from ie_communication.srv import  robotTask, robotTaskResponse
class ProcessController :

    def __init__(self):
        
        self._service = rospy.Service('robot_task', robotTask, self.setTask)
        self._pubGear = rospy.Publisher('/robot_gear', Int32, queue_size=10)
        self._bridge = CvBridge()
        self._camqrFrame = None
        self._currentTask = None
   
    def processing(self):
        pass
    
    def setTask(self, req):
        rospy.loginfo(f"Received task request: {req.task.task_name}")
        if self._currentTask is not None:
            if self._currentTask.running:
                return robotTaskResponse("a task is already running")

        if req.task.task_name == "mapping":
            self._currentTask = Mapping()
            self._currentTask.finishedSignal.connect(self._task_finished)
            self._currentTask.fsignal.connect(self._mapping_finished)
            self._currentTask.start()
        else:
            pass
        return robotTaskResponse("task set")

    def _task_finished(self, message, **kwargs):
        print("Task is finished")
        del self._currentTask
        self._currentTask = None
        rospy.loginfo(message)

    def _task_failed(self, message, **kwargs):
        self._currentTask = None
        rospy.logerr(message)

    def _mapping_finished(self, qrcodes, **kwargs):
        self._currentTask = None

if __name__ == '__main__':
    from threading import Thread

    rospy.init_node('ProcessController')
    pr = ProcessController()
    rospy.spin()