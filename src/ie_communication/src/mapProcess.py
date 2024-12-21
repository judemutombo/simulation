#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
import base64
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import math


class mapProcess :

    def __init__(self):
        self.robot_position = None
        self.robot_rotation = None
        self.pubMap = rospy.Publisher("map_feed", Image, queue_size=10)

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def tf_listener(self):
        try:
            # Lookup the transform from base_link to map
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            
            # Extract position from the transform
            self.robot_position = transform.transform.translation
            self.robot_rotation = transform.transform.rotation
            # rospy.loginfo("Robot position: x=%.2f, y=%.2f", self.robot_position.x, self.robot_position.y)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Could not get transform: %s", str(e))

    def map_callback(self, data):
        rospy.loginfo("Received map")
        if self.robot_position is None or self.robot_rotation is None:
            return  # Wait for the robot's position to be available

        # Create a CvBridge object to convert ROS messages to OpenCV format
        bridge = CvBridge()

        # Convert the OccupancyGrid data to a NumPy array
        # The map data in OccupancyGrid is a 1D list, reshape it into a 2D array (height x width)
        width = data.info.width
        height = data.info.height
        map_data = np.array(data.data, dtype=np.int8).reshape((height, width))

        # Normalize the map values to [0, 255] for OpenCV visualization
        # Typically, values are -1 (unknown), 0 (free), and 100 (occupied)
        map_image = np.zeros_like(map_data, dtype=np.uint8)
        map_image[map_data == 100] = 0  # Occupied cells become white
        map_image[map_data == 0] = 255      # Free cells become black
        map_image[map_data == -1] = 127   # Unknown cells become gray

        
        robot_x_pixel = int((self.robot_position.x - data.info.origin.position.x) / data.info.resolution)
        robot_y_pixel = int((self.robot_position.y - data.info.origin.position.y) / data.info.resolution)
        
        # Scale position to make the robot visible on the map
        scale = 1
        robot_x_pixel *= scale
        robot_y_pixel *= scale

        # Draw a red circle at the robot's position
        map_image = cv2.circle(map_image, (robot_x_pixel, robot_y_pixel), 5, (0, 255, 255), -1)

        # Optionally, draw the robot's orientation (line indicating yaw)
        yaw = self.get_yaw_from_quaternion(self.robot_rotation)  # Use yaw from the transform
        robot_x_end = robot_x_pixel + int(np.cos(yaw) * 10)  # Line length 10
        robot_y_end = robot_y_pixel + int(np.sin(yaw) * 10)

        # Draw the yaw direction as a line
        map_image = cv2.line(map_image, (robot_x_pixel, robot_y_pixel), (robot_x_end, robot_y_end), (0, 255, 0), 2)
        
        # Flip the image vertically (Y-axis flip)
        map_image = np.flipud(map_image)  # Flip the image along the Y-axis

        # Rotate the image by -90 degrees (counterclockwise)
        map_image = cv2.rotate(map_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Save the updated map with robot's position and orientation
        cv2.imwrite("/tmp/robot_map_with_position.png", map_image)
        # rospy.loginfo("Map with robot's position saved")

        # Send the updated map image over SocketIO
        self.send_image("/tmp/robot_map_with_position.png")

    def send_image(self, image_path):
        # Read the image
        img = cv2.imread(image_path)
        _, img_encoded = cv2.imencode('.png', img)

        # Convert the image to base64
        img_base64 = base64.b64encode(img_encoded).decode("utf-8")

        # Publish the image
        bridge = CvBridge()
        self.pubMap.publish(bridge.cv2_to_imgmsg(img, "bgr8"))

    def get_yaw_from_quaternion(self, rotation):
        """
        Extract yaw (rotation around the Z-axis) from a quaternion.
        
        :param rotation: Quaternion (w, x, y, z)
        :return: Yaw angle in radians
        """
        w = rotation.w
        x = rotation.x
        y = rotation.y
        z = rotation.z

        # Calculate yaw from the quaternion using the formula
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        
        yaw = math.atan2(siny, cosy)  # Result is in radians
        return yaw
    
if __name__ == '__main__':

    rospy.init_node('mapProcess', disable_signals=True)
    map_process = mapProcess()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        map_process.tf_listener()  # Get the robot position from tf
        rate.sleep()
    rospy.spin()