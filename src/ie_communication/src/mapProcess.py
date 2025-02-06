#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
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
        self.latest_scan = None
        self.bridge = CvBridge()
        self.path_history = []  # To store the robot's path
        self.qr_code_positions = []  # To store QR code positions
        self.qr_code_size = 3  # Size of the QR code squares (adjustable)

        # Map parameters
        self.map_resolution = 0.05  # 5 cm per pixel
        self.map_width = 400  # Width of the map in pixels
        self.map_height = 400  # Height of the map in pixels
        self.map_origin_x = -10.0  # X coordinate of the map origin (in meters)
        self.map_origin_y = -10.0  # Y coordinate of the map origin (in meters)
        self.map_image = np.ones((self.map_height, self.map_width), dtype=np.uint8) * 255  # Initialize map as free space (white)

        # Publishers and Subscribers
        self.pub_map = rospy.Publisher("map_feed", Image, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)  # Subscribe to /scan

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

    def scan_callback(self, data):
        """
        Callback for the /scan topic. Stores the latest laser scan data.
        """
        self.latest_scan = data
        rospy.loginfo_once("Laser scan data received.")

    def timer_callback(self, event):
        """
        Timer callback for fixed-rate processing of the robot position and laser scan.
        """
        # Get the latest robot position and orientation
        self.tf_listener()

        # Process the laser scan if it's available
        if self.latest_scan is not None and self.robot_position is not None and self.robot_rotation is not None:
            self.process_scan()

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

    def process_scan(self):
        """
        Process the laser scan data and update the local map.
        """
        # Reset map to empty space but do NOT clear previous obstacles
        self.map_image.fill(255)  # Set all pixels to white (free space)

        # Convert robot's position to pixel coordinates
        robot_x_pixel = int((self.robot_position.x - self.map_origin_x) / self.map_resolution)
        robot_y_pixel = int((self.robot_position.y - self.map_origin_y) / self.map_resolution)

        if not (0 <= robot_x_pixel < self.map_width and 0 <= robot_y_pixel < self.map_height):
            rospy.logwarn(f"Robot position ({robot_x_pixel}, {robot_y_pixel}) is out of bounds.")
            return

        # Update map with the latest scan (draw obstacles)
        self.update_map_from_scan()

        # Convert to a color image for visualization
        map_image_color = cv2.cvtColor(self.map_image, cv2.COLOR_GRAY2BGR)

        # Draw the robot's path in green
        if len(self.path_history) > 1:
            for i in range(1, len(self.path_history)):
                cv2.line(map_image_color, self.path_history[i - 1], self.path_history[i], (0, 255, 0), 2)

        # Draw QR code positions in blue
        for qr_x, qr_y in self.qr_code_positions:
            qr_x_pixel = int((qr_x - self.map_origin_x) / self.map_resolution)
            qr_y_pixel = int((qr_y - self.map_origin_y) / self.map_resolution)
            if 0 <= qr_x_pixel < self.map_width and 0 <= qr_y_pixel < self.map_height:
                cv2.rectangle(
                    map_image_color,
                    (qr_x_pixel - self.qr_code_size, qr_y_pixel - self.qr_code_size),
                    (qr_x_pixel + self.qr_code_size, qr_y_pixel + self.qr_code_size),
                    (255, 0, 0),  # Blue
                    -1
                )

        # Draw robot position in red
        cv2.circle(map_image_color, (robot_x_pixel, robot_y_pixel), 5, (0, 0, 255), -1)

        # Draw robot orientation (Yaw Direction)
        yaw = self.get_yaw_from_quaternion(self.robot_rotation)
        robot_x_end = robot_x_pixel + int(np.cos(yaw) * 10)
        robot_y_end = robot_y_pixel + int(np.sin(yaw) * 10)
        cv2.line(map_image_color, (robot_x_pixel, robot_y_pixel), (robot_x_end, robot_y_end), (0, 255, 0), 2)

        # Flip and rotate image for correct orientation
        map_image_color = np.flipud(map_image_color)
        map_image_color = cv2.rotate(map_image_color, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Save and publish map
        output_path = "/tmp/robot_map_with_position.png"
        cv2.imwrite(output_path, map_image_color)
        self.send_image(output_path)


    def update_map_from_scan(self):
        """
        Update the local map based on the latest laser scan data.
        """
        if self.robot_position is None or self.robot_rotation is None:
            return

        # Get the robot's yaw (orientation)
        yaw = self.get_yaw_from_quaternion(self.robot_rotation)

        # Iterate through the laser scan ranges
        for i, range_val in enumerate(self.latest_scan.ranges):
            if range_val < self.latest_scan.range_min or range_val > self.latest_scan.range_max:
                continue  # Skip invalid range values

            # Calculate the angle of the current laser beam
            angle = self.latest_scan.angle_min + i * self.latest_scan.angle_increment

            # Calculate the position of the laser beam endpoint in the map frame
            endpoint_x = self.robot_position.x + range_val * np.cos(yaw + angle)
            endpoint_y = self.robot_position.y + range_val * np.sin(yaw + angle)

            # Convert the endpoint position to pixel coordinates
            endpoint_x_pixel = int((endpoint_x - self.map_origin_x) / self.map_resolution)
            endpoint_y_pixel = int((endpoint_y - self.map_origin_y) / self.map_resolution)
            # Ensure the endpoint is within the map bounds
            if 0 <= endpoint_x_pixel < self.map_width and 0 <= endpoint_y_pixel < self.map_height:
                # Mark the endpoint as occupied (black)
                self.map_image[endpoint_y_pixel, endpoint_x_pixel] = 0

                
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