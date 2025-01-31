
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
        self.fsignal = signalslot.Signal(args=['messages'])
        self.moveToLiftPosition = False
        self.LiftPosition = (0,0)
        self.isLifting = False
    
    def start(self):
        if self._loadQrCodes():
            super().start()
        else:
            super()._finish_task(success=False,message="No Qr code found, you may need to do a mapping first.")

    def _check_qr(self, decoded_text):
        #print(f"position : {self.robot_pose.position.x} {self.robot_pose.position.y}")
        if decoded_text[0] == "None" or decoded_text[0] is None:
            return
        if decoded_text[0] != self._lastQrCode:
            #print(f"QR Code: {decoded_text[0]}")
            self.hasDetectedQrRecently = True
            if (decoded_text[0] in self.qrcodes):
                self._lastQrCode = decoded_text[0]
                self._checkzone(self._lastQrCode) 
            else:
                self._lastQrCode = decoded_text[0]
                self._checkzone(self._lastQrCode) 
            
    def _checkzone(self, qr):
        current = self._params[self._currentTask]
        zArr = current["zone"].split(" ")
        goal = f"{zArr[0]}_{zArr[1]}_center"
        if(goal == qr):
            print("A goal has been reached") 
            position = self._calculate_distance(self.robot_pose)

            print(f"qr: {goal}")
            self.qrcodes[qr] = position
            self.LiftPosition = self._calculate_distance(self.robot_pose)
            print(f"Lift position : {self.LiftPosition}")
            self.moveToLiftPosition = True
            

    def _task_finished(self, message):
        self._running = False
        super()._task_finished(message)
        self.fsignal.emit(messages='done')
    
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
        print(self.hasDetectedQrRecently)
        self.stop()
        tm = 3

        # get the current task
        current = self._params[self._currentTask]
        
        # check if the robot scan a qrCode recently
        if not self.hasDetectedQrRecently: # no qrCode has been scanned

            if onTop: # if not qrCode has been scanned just go straight
                tm = 2
                print("No qrCode has been scanned, going straight")
                self._move_forward()
            else : # if the robot can't go straight, failed the task
                self._task_failed("The robot is stuck : should go straight")

        elif self._lastQrCode : # qrCode has been scanned
            if (self._lastQrCode in self.qrcodes):  # check if the scanned qrCode is in the database
                if self._isStationSide(self._lastQrCode): # check if the scanned qrCode is a station side
                    tm = 2
                    print("qrCode is a station side, going straight")
                    self._move_forward()
                else:
                    if self.goal_in_db(): # check if the robot has the position of the goal zone
                        print("Goal in db")
                        current = self._params[self._currentTask]
                        zArr = current["zone"].split(" ")
                        goal = f"{zArr[0]}_{zArr[1]}_center"
                        goalPos = self.qrcodes.get(goal, (None, None))
                        side = self._chooSideToGo(onTop, onRight, onLeft, goalPos)
                        if side == "S":
                            tm = 2
                            self._move_forward()
                        elif side == "R":
                            tm = 3
                            self._turn_right()
                        elif side == "L":
                            tm = 3
                            self._turn_left()
                        elif side == "B":
                            tm = 6
                            self._U_turn() 
                        elif side == "0":
                            return
                    else:  # the robot doesn't have the position of the goal zone, we refer to the intersection
                        intersections = self._getIntersection(current['zone']) # get the intersections of the goal zone
                        if(self._lastQrCode in [intersections[0][0], intersections[1][0]]): # the last scanned qrCode is one of the goal zone intersection
                            if onLeft: # possibility of going left, so the robot goes left
                                self._turn_left()
                            elif onRight: # possibility of going right so the robot goes right
                                self._turn_right()
                            else: # on intersection the robot should turn, if can't, failed the task 
                                self._task_failed("The robot is stuck : should turn") 
                        else: # the last scanned qrCode is not one of the goal zone intersection
                            # check if the robot is not in front of a corner
                            if onLeft and (not onTop) and (not onRight) : # in front of a corner going left
                                self._turn_left() 
                            elif onRight and (not onTop) and (not onLeft) : # in front of a corner going right
                                self._turn_right()
                            else: # not a corner, use shortest path
                                print("Not a corner, using shortest path")
                                side = self._chooseSide(onTop, onRight, onLeft, intersections)
                                if side == "S":
                                    tm = 2
                                    self._move_forward()
                                elif side == "R":
                                    tm = 3
                                    self._turn_right()
                                elif side == "L":
                                    tm = 3
                                    self._turn_left()
                                elif side == "B":
                                    tm = 6
                                    self._U_turn() 
                                elif side == "0":
                                    return

        self.timer = rospy.Timer(rospy.Duration(tm), self.resume_processing, oneshot=True)

    def _isStationSide(self, qrCode) -> bool:
        zArr = qrCode.split("_")
        return zArr[len(zArr)-1] == "left" or zArr[len(zArr)-1] == "right"

    def _getIntersection(self, zone : str):
        zArr = zone.split(" ")
        inter1 = f"intersection_{zArr[0]}_{zArr[1]}_station_1"
        inter2 = f"intersection_{zArr[0]}_{zArr[1]}_station_2"
        p1 = (inter1, self.qrcodes.get(inter1, (None, None)))
        p2 = (inter2, self.qrcodes.get(inter2, (None, None)))


        return [p1, p2]

    def _chooseSide(self, onTop, onRight, onLeft, intersections):
        directions = {"S": 0, "L": math.pi / 2, "R": -math.pi / 2, "B": math.pi}
        best_direction = None
        min_distance = float('inf')
        pose = self.robot_pose
        robot_position = (pose.position.x, pose.position.y)
        
        # Extract the intersections and handle the case where they might be (None, None)
        yaw = math.atan2(2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
                        1.0 - 2.0 * (pose.orientation.y**2 + pose.orientation.z**2))

        # Check if both intersections are invalid (None, None)
        if all(intersection[1] == (None, None) for intersection in intersections):
            self._task_failed("Both intersections are invalid (None, None)")
            return "O"
        
        for direction, offset in directions.items():
            # Calculate the new yaw after applying the direction offset
            new_yaw = self.normalize_angle(yaw + offset)
            
            # Project the new position based on the new yaw
            projected_position = self.project_position(robot_position, new_yaw)
            
            # Initialize a list to store the distances
            distances = []
            
            for intersection in intersections:
                # Only calculate distance for valid intersections (not (None, None))
                if intersection[1] != (None, None):
                    distance = self.calculate_distance(projected_position, intersection[1])
                    distances.append(distance)

            # If there are valid distances, calculate the minimum distance
            if distances:
                min_proj_distance = min(distances)
                if min_proj_distance < min_distance:
                    min_distance = min_proj_distance
                    best_direction = direction

        # Return the best direction based on the conditions
        if best_direction == "S" and onTop:
            return best_direction
        elif best_direction == "R" and onRight:
            return best_direction
        elif best_direction == "L" and onLeft:
            return best_direction
        elif best_direction == "B" :  # NEW: Allow a 180-degree turn
            return best_direction
        else:
            self._task_failed("Cannot go in the direction found by the shortest path")
            return "O"
    
    def _chooSideToGo(self, onTop, onRight, onLeft, target_position):
        directions = {"S": 0, "L": math.pi / 2, "R": -math.pi / 2, "B": math.pi}
        best_direction = None
        min_distance = float('inf')

        pose = self.robot_pose
        robot_position = (pose.position.x, pose.position.y)

        # Compute robot yaw from quaternion
        yaw = math.atan2(2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
                        1.0 - 2.0 * (pose.orientation.y**2 + pose.orientation.z**2))

        # Check if target position is invalid
        if target_position == (None, None):
            self._task_failed("Target position is invalid (None, None)")
            return "O"

        for direction, offset in directions.items():
            # Calculate new yaw based on the offset
            new_yaw = self.normalize_angle(yaw + offset)

            # Project the new position based on the new yaw
            projected_position = self.project_position(robot_position, new_yaw)

            # Calculate distance to target
            distance = self.calculate_distance(projected_position, target_position)

            if distance < min_distance:
                min_distance = distance
                best_direction = direction

        # Choose the best valid direction
        if best_direction == "S" and onTop:
            return best_direction
        elif best_direction == "R" and onRight:
            return best_direction
        elif best_direction == "L" and onLeft:
            return best_direction
        elif best_direction == "B":  # 180-degree turn is always allowed
            return best_direction
        else:
            self._task_failed("Cannot go in the direction found by the shortest path")
            return "O"

    def _adjust_orientation(self, error, angle, pixels, mask, w_min, h_min):
        
        if not self.isLifting :
            if self.moveToLiftPosition:
                if self._distanceToLifePosition() > 0.005:
                    #print("Moving to lift position, distance to lift position : ", self._distanceToLifePosition())
                    self._move(error)
                    return None
                else:
                    print("Lift position reached")
                    self.stop()
                    self.isLifting = True
                    self._lift()
                return None
            else:
                return super()._adjust_orientation(error, angle, pixels, mask, w_min, h_min)
    
    def _lift(self, duration=3):
        current = self._params[self._currentTask]
        liftType = current["type"]
        start = 0 if liftType == "Loading" else 1
        end = 1 if liftType == "Loading" else 0

        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration.from_sec(duration)
        rate = rospy.Rate(100)  # Set rate to 100 Hz for smoother interpolation
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            # Calculate the current time and interpolation fraction
            current_time = rospy.Time.now()
            elapsed = (current_time - start_time).to_sec()
            fraction = elapsed / duration
            fraction = min(fraction, 1.0)  # Ensure fraction does not exceed 1.0

            # Calculate the interpolated value
            current_value = start + fraction * (end - start)

            # Publish the current value
            self.liftPub.publish(current_value)

            # Sleep to maintain the loop rate
            rate.sleep()

        # Ensure the final value is published
        self.liftPub.publish(end)
        self.isLifting = False
        self.moveToLiftPosition = False

        self._currentTask += 1
        if(self._currentTask == len(self._params)):
               self.stop()
               self._task_finished(message="All goals have been reached")

    def _distanceToLifePosition(self):
        x =  (self.robot_pose.position.x - self.LiftPosition[0]) ** 2
        y = (self.robot_pose.position.y - self.LiftPosition[1]) ** 2
        d = math.sqrt(x + y)
        return d

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
    
    def goal_in_db(self):
        current = self._params[self._currentTask]
        zArr = current["zone"].split(" ")
        goal = f"{zArr[0]}_{zArr[1]}_center"
        print(f"Goal: {goal}")
        self.qrcodes.get(goal) is not None
        return goal in self.qrcodes

if __name__ == '__main__':

    rospy.init_node('mapping')
    cr = Carrying()
    cr.start()
    rospy.spin()