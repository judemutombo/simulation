#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
from mapping import Mapping
from ie_communication.srv import  robotTask, robotTaskResponse, taskMessage, mappingOutput
from ie_communication.msg import QrCode
from ie_communication.srv import robotGear, robotGearResponse
import asyncio

class ProcessController :

    def __init__(self):
        
        self._service = rospy.Service('robot_task', robotTask, self.setTask)
        self._pubProcessState = rospy.Publisher('/ProcessState', String, queue_size=10)
        self._bridge = CvBridge()
        self._camqrFrame = None
        self._currentTask = None

    def processing(self):
        pass
    
    def setTask(self, req):
        rospy.loginfo(f"Received task request: {req.task.task_name}")
        if self._currentTask is not None:
            if self._currentTask.running:
                return robotTaskResponse("a process is already running")

        rospy.wait_for_service('change_gear')
        robot_gear = rospy.ServiceProxy('change_gear', robotGear)
        robot_gear.wait_for_service(10)
        try:
            response = robot_gear(0)
            if response.message:
                print("Gear changed to autonomous")
                if req.task.task_name == "mapping":
                    self._currentTask = Mapping()
                    self._currentTask.fsignal.connect(self._mapping_finished)
                    self._currentTask.finishedSignal.connect(self._task_finished)
                    self._currentTask.start()
                    self._publishProcessState()
                    return robotTaskResponse("Mapping process started")
                else:
                    pass

                return robotTaskResponse("Task set")
            else :
                return robotTaskResponse("Task can not be set")
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return robotTaskResponse("Task can not be set")

    def _task_finished(self, message, **kwargs):
        self._currentTask.stop()
        self._currentTask = None
        rospy.loginfo(f"Message from task : {message}")

        rospy.wait_for_service('output')
        robot_task = rospy.ServiceProxy('output', taskMessage)
        robot_task.wait_for_service(10)
        try:
            response = robot_task(message)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        self._publishProcessState()

    def _task_failed(self, message, **kwargs):
        self._currentTask = None
        rospy.logerr(message)

    def _mapping_finished(self, qrcodes, **kwargs):
        rospy.loginfo("Mapping process finished")

        rospy.wait_for_service('mapping_output')
        mo = rospy.ServiceProxy('mapping_output', mappingOutput)
        mo.wait_for_service(10)
        try:
            key_value_array = [QrCode(location=key, x=value[0], y=value[1]) for key, value in qrcodes.items()]
            response = mo(key_value_array)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def _publishProcessState(self):
        state = "Processing" if self._currentTask is not None else "Stationary"
        self._pubProcessState.publish(state)


if __name__ == '__main__':
    from threading import Thread

    rospy.init_node('ProcessController')
    pr = ProcessController()
    rospy.spin()