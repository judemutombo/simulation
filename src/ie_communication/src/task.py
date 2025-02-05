#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from qreader import QReader
from nav_msgs.msg import Odometry
import math
import signalslot
import asyncio
from sensor_msgs.msg import LaserScan

class Task:

    def __init__(self, task):
        self.robot_pose = None
        rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.scan_data = None
        self.qreader = QReader()
        self.rate = rospy.Rate(10)
        self.msg = Twist()
        self.bridge = CvBridge()
        self.param = {"KP": 500, "SP": 0.08, "TLSP" : 0.07, "TRSP" : 0.07, "MSP" : 1.5}
        self.task = task
        self._running = False

        self.needMakeDecision = False
        self.hasDetectedQrRecently = False
        self._lastQrCode = None

        self.pub = rospy.Publisher("/error", Float32, queue_size=1)
        self.pub2 = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.liftPub = rospy.Publisher("/position_joint_controller/command", Float64, queue_size=10)
        self.stop()
        self.img_size = None
        self._black_pixels = 493850
        self._sideBlackPixels = 414556
        self._threshold = 0.119
        self._middle_width = 580
        self.timer = None
        self.qrcodes = {}
        self.distanceToQr = 0.3869 
        self.finishedSignal = signalslot.Signal(args=['message'])
        self.failedSignal = signalslot.Signal(args=['message'])
        self._processQrCode =  False
        self._making_u_turn = False
        self._making_turn = False
        self._obstacleInFront = False
        self._obstacleChecker = 0
        self._timeToTurn = False
        self.moveToTurnPosition = False
        self._turnSide = None
        rospy.Subscriber("/camera_left/camera_left/image_raw", Image, self.left_callback, queue_size=10)
        rospy.Subscriber("/camera_right/camera_right/image_raw", Image, self.right_callback, queue_size=10)
        rospy.Subscriber("/camera_qr_code_feed",Image, self._camqrProcess)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

    
    def running(self):
        return self._running
    
    def start(self):
        self._running = True
        self.timer = rospy.Timer(rospy.Duration(1), self.check_for_obstacles)  # 10 Hz
        # self._execute_timer_callback()
    
    def _execute_timer_callback(self):
        """
        Callback for the ROS timer. This triggers the asynchronous _execute function.
        """
        if self._running:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._execute())

    def odometry_callback(self, odom_msg):
        self.robot_pose = odom_msg.pose.pose

    def _calculate_distance(self, pose):
        yaw = math.atan2(2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
                         1.0 - 2.0 * (pose.orientation.y**2 + pose.orientation.z**2))
        
        dx  = self.distanceToQr * math.cos(yaw)
        dy  = self.distanceToQr * math.sin(yaw)

        xqr = pose.position.x + dx
        yqr = pose.position.y + dy

        return (xqr, yqr)
      
    def _camqrProcess(self, data):
        if not self._running:
            return
        
        if self.robot_pose is None:
            return
        
        try: 
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            
            decoded_text = self.qreader.detect_and_decode(image=cv_image)
            if decoded_text is not None and len(decoded_text) != 0 and decoded_text[0] != "None":
                # print(f"QR Code: {decoded_text[0]}")
                self.hasDetectedQrRecently = True
                self._check_qr(decoded_text)
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def _check_qr(self, decoded_text):
        pass

    def _move_forward(self):
        """Move forward at a constant speed."""
        self.msg.linear.x = self.param["TLSP"]
        self.msg.angular.z = 0.0
        self.pub2.publish(self.msg)

    def _turn_left(self):
        """Turn the robot to the left."""
        print("Left")
        self.msg.linear.x = 0
        self.msg.angular.z = 0.5
        self.pub2.publish(self.msg)

    def _turn_right(self):
        """Turn the robot to the right."""
        print("Right")
        self.msg.linear.x = 0
        self.msg.angular.z = -0.5
        self.pub2.publish(self.msg)
    
    def _U_turn(self):
        """Turn the robot in 180°."""
        print("U turn")
        self._making_u_turn = True
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.5
        self.pub2.publish(self.msg)
        
    def _move(self, error):
        """Move the robot based on the error."""
        self.msg.linear.x = self.param["SP"]
        self.msg.angular.z = -error/self.param["KP"]  
        self.pub2.publish(self.msg)
    
    def stop(self):
        """Stop the robot."""
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        self.pub2.publish(self.msg)

    def _store(self):
        pass
    
    def _adjust_orientation(self, error, angle, pixels, mask , w_min, h_min):

        """Adjust the robot's orientation based on the error."""
        if self._obstacleInFront:
            return None

        if self.moveToTurnPosition:
            self._move(error)
            return
        
        if self._making_turn:
            print("error", error)
            if error < 60 or error > -60:
                self.stop()
                self._making_turn = False
            return

        if self._making_u_turn:
            print("error", error)
            if error < 60 or error > -60:
                self.stop()
                self._making_u_turn = False
            return

        if (w_min < 100 and h_min < 100):
            self._move_forward()
            return None
        
        ## if the pixels are within the threshold, move forward and adjust the orientation based on the error
        if (pixels <= self._black_pixels + (self._black_pixels * self._threshold)) and  (pixels >= self._black_pixels - (self._black_pixels * self._threshold)):
            self._move(error)
            return None
        
        ## if the pixels are greater than the threshold, check the side pixels
        if pixels > (self._black_pixels + (self._black_pixels * self._threshold)):
            
            left_pixels, right_pixels, onLeft, onRight = self.check_side_pixels(mask)

            ## if the pixels are on the left or right side, check the top pixels
            if onLeft or onRight:
                top_pixels_remaining, top_pixels_side, bottom_pixels_side, onTop, hastoStop = self.check_straight_pixels(mask)
                if hastoStop :
                    self.needMakeDecision = True
                    self.junction_decision(onLeft, onRight, onTop)
                
                if top_pixels_remaining == 0 :
                    self._move_forward()
                return [left_pixels, right_pixels,top_pixels_remaining, top_pixels_side, bottom_pixels_side]
            
            ## if the pixels are not on the left or right side, move forward
            self._move_forward()
            return [left_pixels, right_pixels,0,0,0]
        
        ## if the pixels are less than the threshold, move forward
        elif pixels < (self._black_pixels - (self._black_pixels * self._threshold)):
            self._move(error)
            return None
        
        self._move(error)
        return None 

    def check_side_pixels(self, mask):
        onLeft = False
        onRight = False

        height, width = mask.shape
        middle_start = (width // 2) - (self._middle_width // 2)
        middle_end = (width // 2) + (self._middle_width // 2)

        middle_mask = np.zeros_like(mask)
        middle_mask[:, middle_start:middle_end] = 255 

        masked_binary = cv2.bitwise_and(mask, cv2.bitwise_not(middle_mask))
        left_half = masked_binary[:, :width // 2]
        right_half = masked_binary[:, width // 2:]
        left_pixels = np.sum(left_half == 255)
        right_pixels = np.sum(right_half == 255)
        #print(f"Left pixels: {left_pixels}, Right pixels: {right_pixels}")

        threshold_lr = (self._black_pixels // 3) - 20
        if left_pixels > threshold_lr :
            onLeft = True
        if right_pixels > threshold_lr:
            onRight = True
        
        return left_pixels, right_pixels, onLeft, onRight

    def check_straight_pixels(self, mask):
        onTop = False
        hastoStop = False

        height, width = mask.shape
        middle_start = (width // 2) - (self._middle_width // 2)
        middle_end = (width // 2) + (self._middle_width // 2)

        middle_mask = np.zeros_like(mask)
        middle_mask[:, middle_start:middle_end] = 255 

        masked_binary = cv2.bitwise_and(mask, cv2.bitwise_not(middle_mask))
        top_half = masked_binary[:height // 2, :]
        bottom_half = masked_binary[height // 2:, :]

        top_pixels = np.sum(top_half == 255)
        bottom_pixels = np.sum(bottom_half == 255)
        #print(f"Top pixels: {top_pixels}, Bottom pixels: {bottom_pixels}")

        if bottom_pixels >= top_pixels:
            hastoStop = True
            middle_line = mask[:, middle_start:middle_end]
            height, _ = middle_line.shape
            top_half = middle_line[:height // 2, :]
            bottom_half = middle_line[height // 2:, :]

            bottom_half[bottom_half == 255] = 0
            top_pixels_remaining = np.sum(top_half == 255)
            continuity_threshold = 0.3 * self._black_pixels
            #print(f"Top pixels remaining: {top_pixels_remaining}, Continuity threshold: {continuity_threshold}")
            if top_pixels_remaining >= continuity_threshold:
                onTop = True
                return top_pixels_remaining, top_pixels, bottom_pixels, onTop, hastoStop
            else:
                return top_pixels_remaining, top_pixels, bottom_pixels, onTop, hastoStop
        else:
            return 0, top_pixels, bottom_pixels, onTop, hastoStop

    def junction_decision(self, onLeft, onRight, onTop):
        print("Making decision")
        tm = 3
        if self.task == "mapping":
            if onTop:
                print("On top")
                tm = 1
                self._move_forward()
                self.timer = rospy.Timer(rospy.Duration(tm), self.resume_processing, oneshot=True)
            elif onLeft:
                self._turnSide = "left"
                self.moveToTurnPosition = True
            elif onRight:
                self._turnSide = "right"
                self.moveToTurnPosition = True

    def resume_processing(self, event = None):
        rospy.loginfo("Resuming image processing")
        self.needMakeDecision = False
        self.hasDetectedQrRecently = False
        self.moveToTurnPosition = False
        self._turnSide = None
        self._timeToTurn = False
        if self.timer:
            self.timer.shutdown()  # Clean up the timer

    def callback(self, data):
        if not self._running:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
            black_pixels = np.sum(mask == 255)

            x_last = image.shape[1] / 2
            y_last = image.shape[0] / 2
            self.img_size = image.shape

            x_center = image.shape[1] // 2  # Width center of the image
            y_center = image.shape[0] // 2  # Height center of the image

            # Calculate ROI boundaries
            x_start = max(0, x_center - (self._middle_width)  // 2)
            x_end = min(image.shape[1], x_center + (self._middle_width)  // 2)

            # Set all pixels outside the ROI to a color that won't be detected as black (e.g., white)
            # Create a mask for the pixels outside the ROI
            image_outside_roi = image.copy()

            # Set pixels outside the ROI to white
            image_outside_roi[:, :x_start] = [255, 255, 255]  # Left side
            image_outside_roi[:, x_end:] = [255, 255, 255]    # Right side

            #cv2.imshow("roi",image_outside_roi)

            Blackline = cv2.inRange(image_outside_roi, (0,0,0), (60,60,60))	
            kernel = np.ones((3,3), np.uint8)
            Blackline = cv2.erode(Blackline, kernel, iterations=5)
            Blackline = cv2.dilate(Blackline, kernel, iterations=9)	

            contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            contours_blk_len = len(contours_blk)
            if contours_blk_len > 0 :
                if contours_blk_len == 1 :
                    blackbox = cv2.minAreaRect(contours_blk[0])
                else:
                    # Sort contours by area, in descending order, and pick the largest
                    contours_blk = sorted(contours_blk, key=cv2.contourArea, reverse=True)
                    largest_contour = contours_blk[0]
                    
                    # Get the bounding box of the largest contour
                    blackbox = cv2.minAreaRect(largest_contour)	 
                (x_min, y_min), (w_min, h_min), ang = blackbox
                x_last = x_min
                y_last = y_min
                if ang < -45 :
                    ang = 90 + ang
                if w_min < h_min and ang > 0:	  
                    ang = (90-ang)*-1
                if w_min > h_min and ang < 0:
                    ang = 90 + ang	  
                setpoint = image.shape[1] / 2
                error = int(x_min - setpoint) 
                ang = int(ang)	 
                box = cv2.boxPoints(blackbox)
                box = np.intp(box)
                cv2.drawContours(image,[box],0,(0,0,255),3)	 
                cv2.line(image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
                cv2.putText(image,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(image,str(error),(10, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                cv2.putText(image,str(f"UCL:{self._black_pixels + (self._black_pixels * self._threshold)}"),(10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                cv2.putText(image,str(f"CL:{self._black_pixels}"),(10, 160), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(image,str(f"LCL:{self._black_pixels - (self._black_pixels * self._threshold)}"),(10, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                cv2.putText(image,str(f"BP:{black_pixels}"),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                
                if black_pixels > (self._black_pixels + (self._black_pixels * self._threshold)):
                    cv2.line(image, (image.shape[1] // 2, 0), (image.shape[1] // 2, image.shape[0]), (0, 0, 255), 2)
                    cv2.line(image, (0, image.shape[0] // 2), (image.shape[1], image.shape[0] // 2), (0, 0, 255), 2)

                
                if not self.needMakeDecision :
                    px = self._adjust_orientation(error, ang, black_pixels, mask, w_min, h_min)
                    if px is not None:
                        cv2.putText(image,str(f"Left px:{px[0]}"),(10, 480), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        cv2.putText(image,str(f"Right px:{px[1]}"),(10, 560), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        cv2.putText(image,str(f"Top px:{px[2]}"),(10, 640), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        cv2.putText(image,str(f"Top px side:{px[3]}"),(10, 720), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        cv2.putText(image,str(f"Bot px side:{px[4]}"),(10, 800), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                elif self.moveToTurnPosition:
                    px = self._adjust_orientation(error, ang, black_pixels, mask, w_min, h_min)
                    if px is not None:
                        cv2.putText(image,str(f"Left px:{px[0]}"),(10, 480), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        cv2.putText(image,str(f"Right px:{px[1]}"),(10, 560), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        cv2.putText(image,str(f"Top px:{px[2]}"),(10, 640), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        cv2.putText(image,str(f"Top px side:{px[3]}"),(10, 720), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        cv2.putText(image,str(f"Bot px side:{px[4]}"),(10, 800), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            else:
                if not self._making_u_turn and not self._making_turn:
                    self._move_forward()

           
            
            cv2.imshow('Main', image)
            key = cv2.waitKey(1) & 0xFF	
            if key == ord("q"):
                self.stop()
        except CvBridgeError as e:
            print(e)

    def _finish_task(self, success=True, message=None):
        # Common cleanup actions
        cv2.destroyAllWindows()
        self.stop()
        self._running = False

        # Emit the appropriate signal based on success or failure
        if success:
            self.finishedSignal.emit(message=f"{self.task} process is finished")
        else:
            self.failedSignal.emit(message=f"{self.task} process failed: {message}")

    def _task_finished(self, message):
        self._finish_task(success=True)

    def _task_failed(self, message):
        self._finish_task(success=False, message=message) 
    
    def map_callback(self, data):
        """
        Callback function for the /scan topic. Updates the scan data.
        """
        self.scan_data = None = data  # Store the latest scan data

    def check_for_obstacles(self, event):
        """
        Function to check for obstacles in front of the robot using LIDAR data.
        This function should be called periodically (e.g., in the main loop).
        """
        if self.scan_data is None:
            return  # No LIDAR data available yet

        print("Checking for obstacles using LIDAR data")

        # Define the angle range to check for obstacles (in radians)
        front_angle_range = 30  # Degrees
        front_angle_range_rad = np.deg2rad(front_angle_range)

        # Get the angle increment and number of ranges
        angle_increment = self.scan_data.angle_increment
        num_ranges = len(self.scan_data.ranges)

        # Calculate the indices corresponding to the front angle range
        start_index = int((-front_angle_range_rad / 2 - self.scan_data.angle_min) / angle_increment)
        end_index = int((front_angle_range_rad / 2 - self.scan_data.angle_min) / angle_increment)

        # Check for obstacles in the front angle range
        min_distance = float('inf')
        for i in range(start_index, end_index):
            if 0 < self.scan_data.ranges[i] < min_distance:
                min_distance = self.scan_data.ranges[i]

        # Define the obstacle distance threshold (in meters)
        obstacle_distance_threshold = 0.5  # Adjust this value as needed

        # Check if an obstacle is detected
        if min_distance < obstacle_distance_threshold:
            self._obstacleInFront = True
            rospy.loginfo(f"Obstacle detected at {min_distance:.2f} meters!")
            self.stop()  # Stop the robot

            # check for 5 times while checking if the obstacle is still there
            if self._obstacleChecker < 5:
                if min_distance >= obstacle_distance_threshold:  # Obstacle has moved
                    rospy.loginfo("Obstacle has been moved. Resuming movement.")
                    self._obstacleInFront = False
                    self._obstacleChecker = 0
                    return
                else:
                    self._obstacleChecker += 1
            else:
                # If the obstacle is still there after 5 seconds, call the contour_obstacle function
                rospy.loginfo("Obstacle is still present. Calling contour_obstacle function.")
                self._obstacleChecker = 0
                self.contour_obstacle()
        else:
            self._obstacleInFront = False
            rospy.loginfo("No obstacle detected. Moving forward.")

    def contour_obstacle(self):
        """
        Function to handle obstacle contouring. This is a stub and should be implemented
        with the logic to navigate around the obstacle.
        """
        rospy.loginfo("Contouring obstacle...")
        # Add your logic here to navigate around the obstacle
        pass

    async def _execute(self):
        """
        Main execution loop for the task. This function is called when the task starts.
        """
        try:
            while self._running :
                # await self.check_for_obstacles()  # Check for obstacles asynchronously
                await asyncio.sleep(1)  # Non-blocking sleep to maintain the loop rate
        except rospy.ROSInterruptException:
            rospy.logwarn("ROS is shutting down, stopping obstacle checking.")

    def processSideImage(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY_INV)

            self.img_size = image.shape

            x_center = image.shape[1] // 2  # Width center of the image

            # Calculate ROI boundaries
            x_start = max(0, x_center - (self._middle_width)  // 2)
            x_end = min(image.shape[1], x_center + (self._middle_width)  // 2)

            # Set all pixels outside the ROI to a color that won't be detected as black (e.g., white)
            # Create a mask for the pixels outside the ROI
            image_outside_roi = image.copy()

            # Set pixels outside the ROI to white
            image_outside_roi[:, :x_start] = [255, 255, 255]  # Left side
            image_outside_roi[:, x_end:] = [255, 255, 255]    # Right side

            #cv2.imshow("roi",image_outside_roi)

            Blackline = cv2.inRange(image_outside_roi, (0,0,0), (60,60,60))	
            kernel = np.ones((3,3), np.uint8)
            Blackline = cv2.erode(Blackline, kernel, iterations=5)
            Blackline = cv2.dilate(Blackline, kernel, iterations=9)	

            contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            contours_blk_len = len(contours_blk)
            if contours_blk_len > 0 :
                if contours_blk_len == 1 :
                    blackbox = cv2.minAreaRect(contours_blk[0])
                else:
                    # Sort contours by area, in descending order, and pick the largest
                    contours_blk = sorted(contours_blk, key=cv2.contourArea, reverse=True)
                    largest_contour = contours_blk[0]
                    
                    # Get the bounding box of the largest contour
                    blackbox = cv2.minAreaRect(largest_contour)	 
                (x_min, y_min), (w_min, h_min), ang = blackbox
                if ang < -45 :
                    ang = 90 + ang
                if w_min < h_min and ang > 0:	  
                    ang = (90-ang)*-1
                if w_min > h_min and ang < 0:
                    ang = 90 + ang	  
                setpoint = image.shape[1] / 2
                ang = int(ang)	 
                box = cv2.boxPoints(blackbox)
                box = np.intp(box)
                cv2.drawContours(image,[box],0,(0,0,255),3)	 
            self.shouldTurn(mask)
        except CvBridgeError as e:
            print(e)

    def left_callback(self, data):
        if self._timeToTurn:
            return
        
        if not self.moveToTurnPosition:
            return
        
        if self._turnSide != "left":
            return
        
        self.processSideImage(data)

    def right_callback(self, data):
        if self._timeToTurn:
            return
        
        if not self.moveToTurnPosition:
            return
        
        if self._turnSide != "right":
            return
        self.processSideImage(data)
           

    def shouldTurn(self, mask):
        height, width = mask.shape
        middle_start = (width // 2) - (self._middle_width // 2)
        middle_end = (width // 2) + (self._middle_width // 2)

        middle_mask = np.zeros_like(mask)
        middle_mask[:, middle_end:] = 255 
        middle_mask[:, :middle_start] = 255 

        masked_binary = cv2.bitwise_and(mask, cv2.bitwise_not(middle_mask))
        left_half = masked_binary[:, :width // 2]
        right_half = masked_binary[:, width // 2:]

        bp = np.sum(masked_binary == 255)
        if bp < (self._sideBlackPixels - (self._sideBlackPixels * 0.30)):
            return
        
        left_pixels = np.sum(left_half == 255)
        right_pixels = np.sum(right_half == 255)
        rospy.loginfo(f"Left pixels: {left_pixels}, Right pixels: {right_pixels}")
        if left_pixels > right_pixels :
            print(f"Side : {self._turnSide}")
            self.moveToTurnPosition = False
            self.stop()
            self._timeToTurn = True
            self.timer = rospy.Timer(rospy.Duration(1), self.turn, oneshot=True)
            
    def turn(self, event = None):
        self._making_turn = True
        if self._turnSide == "left":
            self._turn_left()
        elif self._turnSide == "right":
            self._turn_right()
        
        self.timer = rospy.Timer(rospy.Duration(2), self.resume_processing, oneshot=True)

if __name__ == '__main__':

    rospy.init_node('Threshold')
    th = Task(task="mapping")
    th.start()
    rospy.spin()
    