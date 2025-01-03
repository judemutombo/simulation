#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2



class Movement:
    def __init__(self):
        print("Movement node started")
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self._callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()
        self.msg = Twist()
        self.integral = self.prev_error = 0
        self.param = {"KP": 0.0046, "KI": 0.0096, "KD": 0.78, "SP": 0.2}
        self.junction_detected = False
        self.is_line_detected = False

    def _move_forward(self):
        """Move forward at a constant speed."""
        self.msg.linear.x = self.param["SP"]
        self.msg.angular.z = 0.0
        self.pub.publish(self.msg)

    def _stop(self):
        """Stop the robot."""
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        self.pub.publish(self.msg)

    def _adjust_orientation(self, error):
        """Adjust the robot's orientation based on the error."""
        
        self.msg.linear.x = self.param["SP"]
        self.msg.angular.z = -self.param["KP"] * error
        self.pub.publish(self.msg)

    def _callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            Blackline = cv2.inRange(image, (0,0,0), (60,60,60))	
            kernel = np.ones((3,3), np.uint8)
            Blackline = cv2.erode(Blackline, kernel, iterations=5)
            Blackline = cv2.dilate(Blackline, kernel, iterations=9)	
            contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

            contours_blk_len = len(contours_blk)
            if contours_blk_len > 0 :
                if contours_blk_len == 1 :
                    blackbox = cv2.minAreaRect(contours_blk[0])
                else:
                    canditates=[]
                    off_bottom = 0	   
                    for con_num in range(contours_blk_len):		
                        blackbox = cv2.minAreaRect(contours_blk[con_num])
                        (x_min, y_min), (w_min, h_min), ang = blackbox		
                        box = cv2.boxPoints(blackbox)
                        (x_box,y_box) = box[0]
                        if y_box > 358 :		 
                            off_bottom += 1
                        canditates.append((y_box,con_num,x_min,y_min))		
                canditates = sorted(canditates)
                if off_bottom > 1:	    
                    canditates_off_bottom=[]
                    for con_num in range ((contours_blk_len - off_bottom), contours_blk_len):
                        (y_highest,con_highest,x_min, y_min) = canditates[con_num]		
                        total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5
                        canditates_off_bottom.append((total_distance,con_highest))
                        canditates_off_bottom = sorted(canditates_off_bottom)         
                        (total_distance,con_highest) = canditates_off_bottom[0]         
                        blackbox = cv2.minAreaRect(contours_blk[con_highest])	   
                else:		
                    (y_highest,con_highest,x_min, y_min) = canditates[contours_blk_len-1]		
                    blackbox = cv2.minAreaRect(contours_blk[con_highest])	 
                (x_min, y_min), (w_min, h_min), ang = blackbox
                x_last = x_min
                y_last = y_min
                if ang < -45 :
                    ang = 90 + ang
                if w_min < h_min and ang > 0:	  
                    ang = (90-ang)*-1
                if w_min > h_min and ang < 0:
                    ang = 90 + ang	  
                setpoint = 320
                error = int(x_min - setpoint) 
                ang = int(ang)	 
                box = cv2.boxPoints(blackbox)
                box = np.int0(box)
                cv2.drawContours(image,[box],0,(0,0,255),3)	 
                cv2.putText(image,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(image,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                cv2.line(image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
                
            cv2.imshow("orginal with line", image)

        except CvBridgeError as e:
            print(e)
            rospy.signal_shutdown("shutdown")
  


if __name__ == '__main__':
    rospy.init_node('movement')
    move = Movement()
    rospy.spin()