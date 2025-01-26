
import rospy
from task import Task
import signalslot
import sqlite3
import os

class Mapping(Task):

    def __init__(self):
        super().__init__("mapping")
        self.fsignal = signalslot.Signal(args=['qrcodes'])

    def start(self):
        if self._checkExistingQrCodes():
            self.fsignal.emit(qrcodes=self.qrcodes)
            super()._finish_task(success=True, message="Mapping process finished")
        else:
            super().start()


    def _check_qr(self, decoded_text):
        if not self._processQrCode :
            self._processQrCode = True
            if decoded_text[0] == "None" or decoded_text[0] is None:
                self._processQrCode = False
                return
            
            if decoded_text[0] != self._lastQrCode:
                print(f"QR Code: {decoded_text[0]}")
                if len(self.qrcodes) >= 1:
                    if (decoded_text[0] == list(self.qrcodes.keys())[0]):
                        self._processQrCode = False
                        self._task_finished(message="Mapping process finished")
                        return
                
                self.hasDetectedQrRecently = False
                position = self._calculate_distance(self.robot_pose)
                self.qrcodes[decoded_text[0]] = position
                print(f"Position: {position}")
                self._lastQrCode = decoded_text[0]
                self._processQrCode = False
            self._processQrCode = False

    def _task_finished(self, message):
        self._running = False
        self._store()
        super()._task_finished(message)
        self.fsignal.emit(qrcodes=self.qrcodes)

    def _store(self):
        connection = sqlite3.connect("qr_code.db")
        cursor = connection.cursor()
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS qr_code (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL,
            position_x INTEGER NOT NULL,
            position_y INTEGER NOT NULL
        )
        """)
        for key, (position_x, position_y) in self.qrcodes.items():
            cursor.execute("""
            INSERT INTO qr_code (name, position_x, position_y)
            VALUES (?, ?, ?)
            """, (key, position_x, position_y))

        # Commit changes and close the connection
        connection.commit()
        connection.close()
    
    def _checkExistingQrCodes(self):
        try:
            connection = sqlite3.connect("qr_code.db")
            cursor = connection.cursor()
            cursor.execute("SELECT name, position_x, position_y FROM qr_code")
            rows = cursor.fetchall()
            for row in rows:
                self.qrcodes[row[0]] = (row[1], row[2])
            connection.close()
            return len(rows) > 0
        except :
            return False
    
if __name__ == '__main__':

    rospy.init_node('mapping')
    mapping = Mapping()
    mapping.start()
    rospy.spin()