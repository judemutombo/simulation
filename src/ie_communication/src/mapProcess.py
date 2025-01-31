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
import sqlite3


class MapProcess:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('map_process', disable_signals=True)

        # Initialize variables
        self.robot_position = None
        self.robot_rotation = None
        self.latest_map = None
        self.bridge = CvBridge()
        self.path_history = []  # To store the robot's path
        self.qr_code_positions = []  # To store QR code positions
        self.qr_code_size = 3  # Size of the QR code squares (adjustable)

        # Publishers and Subscribers
        self.pub_map = rospy.Publisher("map_feed", Image, queue_size=10)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Timer for fixed-rate processing
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)  # 10 Hz

        # Timer for reading QR code positions every 5 seconds
        self.qr_timer = rospy.Timer(rospy.Duration(5), self.read_qr_codes)

    def tf_listener(self):
        """
        Get the robot's position and orientation from the TF tree.
        """
        try:
            # Lookup the transform from base_link to map
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))

            # Extract position and rotation from the transform
            self.robot_position = transform.transform.translation
            self.robot_rotation = transform.transform.rotation
            rospy.loginfo_once("Robot position and rotation updated.")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Could not get transform: %s", str(e))

    def map_callback(self, data):
        """
        Callback for the /map topic. Stores the latest map data.
        """
        self.latest_map = data
        rospy.loginfo_once("Map data received.")

    def timer_callback(self, event):
        """
        Timer callback for fixed-rate processing of the map and robot position.
        """
        # Get the latest robot position and orientation
        self.tf_listener()

        # Process the map if it's available
        if self.latest_map is not None and self.robot_position is not None and self.robot_rotation is not None:
            self.process_map(self.latest_map)

    def read_qr_codes(self, event):
        """
        Read QR code positions from the SQLite database every 5 seconds.
        """
        try:
            # Connect to the SQLite database
            conn = sqlite3.connect('qr_code.db')
            cursor = conn.cursor()

            # Fetch QR code positions from the database
            cursor.execute("SELECT position_x, position_y FROM qr_code")
            self.qr_code_positions = cursor.fetchall()  # Store positions in an array

            # Close the database connection
            conn.close()

            rospy.loginfo(f"Read {len(self.qr_code_positions)} QR code positions from the database.")

        except sqlite3.Error as e:
            rospy.logerr(f"Error reading QR code positions from database: {e}")

    def process_map(self, map_data):
        """
        Process the map data and overlay the robot's position, orientation, path, and QR codes.
        """
        # Convert the OccupancyGrid data to a NumPy array
        width = map_data.info.width
        height = map_data.info.height
        map_array = np.array(map_data.data, dtype=np.int8).reshape((height, width))

        # Normalize the map values to [0, 255] for OpenCV visualization
        map_image = np.zeros_like(map_array, dtype=np.uint8)
        map_image[map_array == 100] = 0    # Occupied cells (black)
        map_image[map_array == 0] = 255    # Free cells (white)
        map_image[map_array == -1] = 127   # Unknown cells (gray)

        # Convert the grayscale map image to a 3-channel color image
        map_image_color = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)

        # Convert robot's position from map frame to pixel coordinates
        robot_x_pixel = int((self.robot_position.x - map_data.info.origin.position.x) / map_data.info.resolution)
        robot_y_pixel = int((self.robot_position.y - map_data.info.origin.position.y) / map_data.info.resolution)

        # Ensure the robot's position is within the map bounds
        if not (0 <= robot_x_pixel < width and 0 <= robot_y_pixel < height):
            rospy.logwarn(f"Robot position ({robot_x_pixel}, {robot_y_pixel}) is outside the map bounds.")
            return

        # Store the robot's current position in the path history
        self.path_history.append((robot_x_pixel, robot_y_pixel))

        # Draw the robot's path as a green line
        if len(self.path_history) > 1:
            for i in range(1, len(self.path_history)):
                cv2.line(map_image_color, self.path_history[i - 1], self.path_history[i], (0, 255, 0), 2)  # Green line

        # Draw QR code positions as blue squares
        for qr_x, qr_y in self.qr_code_positions:
            qr_x_pixel = int((qr_x - map_data.info.origin.position.x) / map_data.info.resolution)
            qr_y_pixel = int((qr_y - map_data.info.origin.position.y) / map_data.info.resolution)

            # Ensure the QR code position is within the map bounds
            if 0 <= qr_x_pixel < width and 0 <= qr_y_pixel < height:
                # Draw a smaller blue square
                cv2.rectangle(
                    map_image_color,
                    (qr_x_pixel - self.qr_code_size, qr_y_pixel - self.qr_code_size),  # Top-left corner
                    (qr_x_pixel + self.qr_code_size, qr_y_pixel + self.qr_code_size),  # Bottom-right corner
                    (255, 0, 0),  # Blue color
                    -1  # Fill the square
                )

        # Draw a red circle at the robot's current position
        map_image_color = cv2.circle(map_image_color, (robot_x_pixel, robot_y_pixel), 5, (0, 0, 255), -1)  # Red circle

        # Optionally, draw the robot's orientation (line indicating yaw)
        yaw = self.get_yaw_from_quaternion(self.robot_rotation)  # Use yaw from the transform
        robot_x_end = robot_x_pixel + int(np.cos(yaw) * 10)  # Line length 10
        robot_y_end = robot_y_pixel + int(np.sin(yaw) * 10)

        # Draw the yaw direction as a line
        map_image_color = cv2.line(map_image_color, (robot_x_pixel, robot_y_pixel), (robot_x_end, robot_y_end), (0, 255, 0), 2)  # Green line

        # Flip the image vertically (Y-axis flip)
        map_image_color = np.flipud(map_image_color)  # Flip the image along the Y-axis

        # Rotate the image by -90 degrees (counterclockwise)
        map_image_color = cv2.rotate(map_image_color, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Save the updated map with robot's position, orientation, path, and QR codes
        output_path = "/tmp/robot_map_with_position.png"
        cv2.imwrite(output_path, map_image_color)
        rospy.loginfo(f"Map with robot's position, path, and QR codes saved to {output_path}")

        # Send the updated map image over ROS
        self.send_image(output_path)

    def send_image(self, image_path):
        """
        Send the processed map image over ROS.
        """
        # Read the image
        img = cv2.imread(image_path)

        # Publish the image
        img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.pub_map.publish(img_msg)
        rospy.loginfo("Map image published.")

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
    try:
        map_process = MapProcess()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass