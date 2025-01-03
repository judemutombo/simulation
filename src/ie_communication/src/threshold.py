#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from qreader import QReader

class Threshold:
    def __init__(self):
        rospy.init_node("Threshold", anonymous=True)
        self.qreader = QReader()
        self.rate = rospy.Rate(10)
        self.msg = Twist()
        self.bridge = CvBridge()
        self.param = {"KP": 0.0046, "SP": 0.05}

        self.decisionMade = False
        self.hasDetectedQrRecently = False

        self._subcamqr = rospy.Subscriber("/camera_qr_code_feed",Image, self._camqrProcess)
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.pub = rospy.Publisher("/error", Float32, queue_size=1)
        self.pub2 = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        

    def _camqrProcess(self, data):
        try: 
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            
            decoded_text = self.qreader.detect_and_decode(image=cv_image)
            if decoded_text is not None and len(decoded_text) != 0:
                print(f"Decoded text: {decoded_text}")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def _move_forward(self):
        """Move forward at a constant speed."""
        self.msg.linear.x = self.param["SP"]
        self.msg.angular.z = 0.0
        self.pub2.publish(self.msg)


    def _adjust_orientation(self, error):
        """Adjust the robot's orientation based on the error."""
        
        self.msg.linear.x = self.param["SP"]
        self.msg.angular.z = -self.param["KP"] * error
        self.pub2.publish(self.msg)

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            x_last = image.shape[1] / 2
            y_last = image.shape[0] / 2
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
                setpoint = image.shape[1] / 2
                error = int(x_min - setpoint) 
                ang = int(ang)	 
                box = cv2.boxPoints(blackbox)
                box = np.intp(box)
                cv2.drawContours(image,[box],0,(0,0,255),3)	 
                cv2.putText(image,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(image,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                cv2.putText(image,str(f"h:{h_min}"),(10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(image,str(f"w:{w_min}"),(10, 160), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                cv2.line(image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)

                top_right = box[1]  # Typically the second point (top-right)
                bottom_left = box[3]  # Typically the fourth point (bottom-left)
                cv2.line(image, tuple(top_right), tuple(bottom_left), (0, 255, 255), 3)

                self._adjust_orientation(error)
            else:
                self._move_forward()

            cv2.imshow('Main', image)
            
        except CvBridgeError as e:
            print(e)
        if cv2.waitKey(1) == 27:
            rospy.signal_shutdown("shutdown")
            cv2.destroyAllWindows()




if __name__ == '__main__':
    obj = Threshold()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)