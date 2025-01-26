#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
from mapping import Mapping
from carrying import Carrying
from ie_communication.srv import  robotTask, robotTaskResponse, taskMessage, mappingOutput
from ie_communication.msg import QrCode
from ie_communication.srv import robotGear, robotGearResponse


class ProcessController :

    def __init__(self):
        
        self._service = rospy.Service('robot_task', robotTask, self.setTask)
        self._pubProcessState = rospy.Publisher('/ProcessState', String, queue_size=10)
        self._bridge = CvBridge()
        self._currentTask = None
        self._proccessing = False

    def processing(self):
        pass
    
    def setTask(self, req):
        rospy.loginfo(f"Received task request: {req.task.task_name}")
        if self._proccessing == True and self._currentTask is not None:
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
                    self._currentTask.failedSignal.connect(self._task_failed)
                    self._currentTask.start()
                    self._publishProcessState()
                    self._proccessing = True
                    return robotTaskResponse("Mapping process started")
                
                elif req.task.task_name == "carrying":
                    params = [{"zone" :kv.zone, "type": kv.type} for kv in req.task.params]
                    self._currentTask = Carrying(params)
                    self._currentTask.fsignal.connect(self._carrying_finished)
                    self._currentTask.finishedSignal.connect(self._task_finished)
                    self._currentTask.failedSignal.connect(self._task_failed)
                    self._currentTask.start()
                    self._publishProcessState()
                    self._proccessing = True
                    return robotTaskResponse("Carrying process started")

                return robotTaskResponse("Task can not be set")
            else :
                return robotTaskResponse("Task can not be set")
            
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return robotTaskResponse("Task can not be set")

    def _task_finished(self, message, **kwargs):
        self._proccessing = False
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
        rospy.loginfo(f"Message from task : {message}")
        self._proccessing = False
        self._currentTask.stop()
        self._currentTask = None
        rospy.wait_for_service('output')
        robot_task = rospy.ServiceProxy('output', taskMessage)
        robot_task.wait_for_service(10)
        try:
            response = robot_task(message)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        rospy.logerr(message)

    def _mapping_finished(self, qrcodes, **kwargs):
        self._proccessing = False
        self._currentTask.stop()
        self._currentTask = None
        rospy.loginfo("Mapping process finished")

        rospy.wait_for_service('mapping_output')
        mo = rospy.ServiceProxy('mapping_output', mappingOutput)
        mo.wait_for_service(10)
        try:
            key_value_array = [QrCode(location=key, x=value[0], y=value[1]) for key, value in qrcodes.items()]
            response = mo(key_value_array)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def _carrying_finished(self, messages, **kwargs):
        self._proccessing = False
        self._currentTask.stop()
        self._currentTask = None
        rospy.loginfo("Carrying process finished")

    def _publishProcessState(self):
        state = "Processing" if self._currentTask is not None else "Stationary"
        self._pubProcessState.publish(state)


if __name__ == '__main__':
    rospy.init_node('ProcessController')
    pr = ProcessController()
    rospy.spin()