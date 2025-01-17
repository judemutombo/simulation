
import rospy
from task import Task
import signalslot
import sqlite3
import math

class Carrying(Task):

    def __init__(self, params):
        self._params = params
        super().__init__("carrying")
        self._currentTask = 0
        self._serializeQr = [()]
    
    def start(self):
        if self._loadQrCodes():
            super().start()
        else:
            super()._task_failed("No Qr code found, you may need to do a mapping first.")

    def _check_qr(self, decoded_text):
        if decoded_text[0] == "None" or decoded_text[0] is None:
            return
        if decoded_text[0] != self._lastQrCode:
            #print(f"QR Code: {decoded_text[0]}")
            if (decoded_text[0] in self.qrcodes):
                self._lastQrCode = decoded_text[0]
            else:
                self._lastQrCode = decoded_text[0]
                self._checkzone(self._lastQrCode) 

    def _checkzone(self, qr):
        current = self._params[self._currentTask]
        zArr = current["zone"].split(" ")
        goal = f"{zArr[0]}_{zArr[1]}_center"
        if(goal == qr):
           self._currentTask += 1
           print("A goal has been reached") 
           if(self._currentTask == len(self._params)):
               self._task_finished()

    def _task_finished(self):
        self._running = False
        super()._task_finished()
    
    def _loadQrCodes(self):
        try:
            connection = sqlite3.connect("qr_code.db")
            cursor = connection.cursor()
            cursor.execute("SELECT name, position_x, position_y FROM qr_code")
            rows = cursor.fetchall()
            for row in rows:
                self.qrcodes[row[0]] = (row[1], row[2])
            connection.close()
            self._serializeQr = [(key,value[0],value[1]) for key, value in self.qrcodes.items()]
            return len(rows) > 0
        except :
            print('no qr code found')
            return False
    
    def junction_decision(self, onLeft, onRight, onTop):
            print("Making decision")
            self.stop()
            tm = 3

            # get the current task
            current = self._params[self._currentTask]
            
            # check if the robot scan a qrCode recently
            if self._lastQrCode is None: # no qrCode has been scanned

                if onTop: # if not qrCode has been scanned just go straight
                    tm = 1
                    self._move_forward()
                else : # if the robot can't go straight, failed the task
                    self._task_failed("The robot is stuck")

            elif self._lastQrCode : # qrCode has been scanned
                if (self._lastQrCode in self.qrcodes):  # check if the scanned qrCode is in the database
                    intersections = self._getIntersection(current['zone']) # get the intersections of the goal zone
                    if(self._lastQrCode in [intersections[0][0], intersections[1][0]]): # the last scanned qrCode is one of the goal zone intersection
                        if onLeft: # possibility of going left, so the robot goes left
                            self._turn_left()
                        elif onRight: # possibility of going right so the robot goes right
                            self._turn_right()
                        else: # on intersection the robot should turn, if can't, failed the task 
                            self._task_failed("The robot is stuck")
                    else: # the last scanned qrCode is not one of the goal zone intersection
                        # check if the robot is not in front of a corner
                        if onLeft and (not onTop) and (not onRight) : # in front of a corner going left
                            self._turn_left() 
                        elif onRight and (not onTop) and (not onLeft) : # in front of a corner going right
                            self._turn_right()
                        else: # not a corner, use shortest path
                            side = self._chooseSide(onTop, onRight, onLeft, intersections)
                            if side == "S":
                                tm = 1
                                self._move_forward()
                            elif side == "R":
                                tm = 3
                                self._turn_right()
                            elif side == "L":
                                tm = 3
                                self._turn_left() 
                            elif side == "0":
                                return

            self.timer = rospy.Timer(rospy.Duration(tm), self.resume_processing, oneshot=True)

    def _getIntersection(self, zone : str):
        zArr = zone.split(" ")
        inter1 = f"intersection_{zArr[0]}_{zArr[1]}_station_1"
        inter2 = f"intersection_{zArr[0]}_{zArr[1]}_station_2"
        p1 = (inter1, (self.qrcodes[inter1][0],self.qrcodes[inter1][1]))
        p2 = (inter2, (self.qrcodes[inter2][0],self.qrcodes[inter2][1]))

        return [p1, p2]

    def _chooseSide(self, onTop, onRight, onLeft, intersections):
        directions = {"S": 0, "L": math.pi / 2, "R": -math.pi / 2}
        best_direction = None
        min_distance = float('inf')
        pose = self.robot_pose
        robot_position = (pose.position.x, pose.position.y)
        intersection1 = intersections[0][1]
        intersection2 = intersections[0][1]
        yaw = yaw = math.atan2(2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
                         1.0 - 2.0 * (pose.orientation.y**2 + pose.orientation.z**2))
        
        for direction, offset in directions.items():
            # Calculate the new yaw after applying the direction offset
            new_yaw = self.normalize_angle(yaw + offset)
            
            # Project the new position based on the new yaw
            projected_position = self.project_position(robot_position, new_yaw)
            
            # Calculate distances to the intersections
            distance1 = self.calculate_distance(projected_position, intersection1)
            distance2 = self.calculate_distance(projected_position, intersection2)
            
            # Choose the smallest distance
            min_proj_distance = min(distance1, distance2)
            if min_proj_distance < min_distance:
                min_distance = min_proj_distance
                best_direction = direction

        if best_direction == "S" and onTop:
            return best_direction
        elif best_direction == "R" and onRight:
            return best_direction
        elif best_direction == "L" and onLeft:
            return best_direction
        else:
            self._task_failed("Cannot go in the direction found by the shortest path")
            return "O"
    
    def project_position(self, position, yaw):
        x, y = position
        # Move 1 unit in the direction of the yaw
        new_x = x + math.cos(yaw)
        new_y = y + math.sin(yaw)
        return (new_x, new_y)

    def calculate_distance(self, pos1, pos2):
        # Use Euclidean distance
        return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5

    def normalize_angle(self, angle):
        # Normalize angle to be between -π and π
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


if __name__ == '__main__':

    rospy.init_node('mapping')
    cr = Carrying()
    cr.start()
    rospy.spin()