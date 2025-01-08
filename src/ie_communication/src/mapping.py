
import rospy
from task import Task
import signalslot

class Mapping(Task):

    def __init__(self):
        super().__init__("mapping")
        self.fsignal = signalslot.Signal(args=['qrcodes'])



    def _check_qr(self, decoded_text):
        if decoded_text[0] == "None" or decoded_text[0] is None:
            return
        if decoded_text[0] != self._lastQrCode:
            if (decoded_text[0] in self.qrcodes):
                super()._task_finished()
                return
            
            self._lastQrCode = decoded_text[0]
            self.hasDetectedQrRecently = True
            position = self._calculate_distance(self.robot_pose)
            self.qrcodes[decoded_text[0]] = position
            print(f"QR Code: {decoded_text[0]}")
            print(f"Position: {position}")

    def _task_finished(self):
        super()._task_finished()
        self.fsignal.emit(qrcodes=self.qrcodes)


if __name__ == '__main__':

    rospy.init_node('mapping')
    mapping = Mapping()
    mapping.start()
    rospy.spin()