#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ie_communication.cfg import PIDControlConfig
from dynamic_reconfigure.server import Server


class Movement:
    def __init__(self):
        print("Movement node started")
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self._callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()
        self.msg = Twist()
        self.param = {"KP": 0.0046, "KI": 0.0096, "KD": 0.78, "SP": 0.2, "ROI_W": 608.0, "ROI_H": 152.0, "ROI_Y": 715}
        self.integral = self.prev_error = 0
        self.t_junction_detected = False
        self.is_line_detected = False
        srv = Server(PIDControlConfig, self._reconfig)

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
        
        self.msg.linear.x = 0.0
        self.msg.angular.z = -self.param["KP"] * error
        self.pub.publish(self.msg)

    def _callback(self, data):
        try:
            input_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            rospy.signal_shutdown("shutdown")

        if not self.t_junction_detected:
            error, is_t_junction, is_line_detected = self._compute_error(input_img)
            print(f"Error: {error}, T-junction: {is_t_junction}, Line detected: {is_line_detected}")
            if not is_line_detected:
                self._move_forward()
            elif not self.is_line_detected:
                self.is_line_detected = is_line_detected
                self._adjust_orientation(error)


            if is_t_junction:
                self.t_junction_detected = True
                self._handle_t_junction()
            else:
                rectify = self._pid(error)
                self.msg.linear.x = self.param["SP"]
                self.msg.angular.z = rectify
                self.pub.publish(self.msg)
        self.rate.sleep()

    def _reconfig(self, config, level):
        self.param = config
        return config

    def _compute_error(self, img_in):
        roi_y = self.param["ROI_Y"]
        roi_h = self.param["ROI_H"]
        roi_w = self.param["ROI_W"]
    
        if roi_w > img_in.shape[1]:
            rospy.loginfo("ROI_W surpassed the image bounds")
            roi_w = img_in.shape[1]
        if (roi_y + (roi_h / 2)) > img_in.shape[0]:
            rospy.loginfo("ROI_Y surpassed the image bounds")
            roi_y = img_in.shape[0] - (roi_h / 2)

        roi = img_in[int(roi_y - (roi_h / 2)):int(roi_y + (roi_h / 2)),
              int((img_in.shape[1] / 2) - (roi_w / 2)):int((img_in.shape[1] / 2) + (roi_w / 2))]
        hsv_img = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        masked = cv2.inRange(hsv_img, np.array([0, 0, 0]), np.array([0, 0, 65]))
        masked = cv2.dilate(masked, np.ones((3, 3), dtype=np.uint8), iterations=5)

        # Check for contours
        contours, _ = cv2.findContours(masked.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x: cv2.contourArea(x))

        is_t_junction = False
        balance = 0
        is_line_detected = self.is_line_detected
        if len(contours) > 0:
            x, y, w, h = cv2.boundingRect(contours[-1])
            cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 0, 255), 2)
            balance = (roi.shape[1] / 2 - (x + (w / 2)))

            # Check for T-junction condition (lines to left and/or right)
            left_region = np.sum(masked[:, :roi.shape[1] // 3])
            right_region = np.sum(masked[:, 2 * roi.shape[1] // 3:])
            center_region = np.sum(masked[:, roi.shape[1] // 3:2 * roi.shape[1] // 3])

            if center_region < 5000 and (left_region > 5000 or right_region > 5000):
                is_t_junction = True
            is_line_detected = True
        else:
            balance = self.prev_error if self.prev_error > 0 else -roi.shape[1] / 2
            is_line_detected = False

        return balance, is_t_junction, is_line_detected

    def _handle_t_junction(self):
        self._stop()
        print("T-junction detected! Please choose an action:")
        print("Options: 'left', 'right', 'straight' (if from middle) or 'turn', 'straight' (if from side).")
        decision = input("Enter your choice: ").strip().lower()

        if decision == "left":
            self.msg.linear.x = 0
            self.msg.angular.z = 1.0  # Adjust angular velocity for left turn
        elif decision == "right":
            self.msg.linear.x = 0
            self.msg.angular.z = -1.0  # Adjust angular velocity for right turn
        elif decision == "straight":
            self.msg.linear.x = self.param["SP"]
            self.msg.angular.z = 0
        elif decision == "turn":
            self.msg.linear.x = 0
            self.msg.angular.z = 1.0  # Assume turn left by default for side scenario
        else:
            print("Invalid choice. Stopping robot for safety.")
            self.msg.linear.x = 0
            self.msg.angular.z = 0

        self.pub.publish(self.msg)
        rospy.sleep(2)  # Allow time for the turn/straight movement
        self.t_junction_detected = False  # Reset the state

    def _pid(self, err):
        self.integral += err
        diff = err - self.prev_error
        self.prev_error = err
        return (self.param["KP"] * err) + (self.param["KI"] * 0.01 * self.integral) + (self.param["KD"] * 0.01 * diff)



if __name__ == '__main__':
    rospy.init_node('movement')
    move = Movement()
    rospy.spin()