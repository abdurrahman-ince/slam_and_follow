#!/usr/bin/env python

"""follower_ros.py: TurtleBot3 will follow the Green Line and stop when the line ends"""

__author__ = "Arjun S Kumar"

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # Subscribe to the camera image topic for TurtleBot3 (use the appropriate topic for your robot)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        # Publish velocity commands to the robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        # Convert the ROS image message to an OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert the image to HSV color space for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds of the green color in HSV
        lower_green = np.array([35, 50, 50])  # Lower bound of green in HSV
        upper_green = np.array([85, 255, 255])  # Upper bound of green in HSV
        
        # Create a mask for the green areas in the image
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Define the region of interest (ROI) where the green line should be
        h, w, d = image.shape
        search_top = int(3 * h / 4)  # Ensure search_top is an integer
        search_bot = int(3 * h / 4 + 20)  # Ensure search_bot is an integer
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        
        # Find moments of the mask to calculate the center of the green line
        M = cv2.moments(mask)
        
        # If there is a green line detected
        if M['m00'] > 0:
            # Calculate the center of the green line
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Draw a circle at the center of the green line for visualization
            cv2.circle(image, (cx, cy), 10, (0, 0, 255), -1)

            # Calculate the error from the center of the image
            error = cx - w / 2

            # Apply a proportional controller for smooth movement
            self.twist.linear.x = 0.2  # Constant speed for forward motion

            # Adjust the angular velocity based on the error
            self.twist.angular.z = -float(error) / 100  # Smooth turning based on the error

            # Apply limits to the angular velocity to prevent sharp turns
            if self.twist.angular.z > 0.5:
                self.twist.angular.z = 0.5
            elif self.twist.angular.z < -0.5:
                self.twist.angular.z = -0.5

            # Apply stronger corrections if the error is large (if the robot is far from the center)
            if abs(error) > 50:  # Adjust this threshold as needed
                self.twist.linear.x = 0.1  # Slow down if the robot is far from the center
                self.twist.angular.z = self.twist.angular.z * 1.5  # Increase turning speed for correction

            # Publish the velocity command
            self.cmd_vel_pub.publish(self.twist)
        else:
            # If no green line is detected, stop the robot
            self.twist.linear.x = 0.0  # Stop the robot's forward movement
            self.twist.angular.z = 0.0  # Stop the robot's rotation
            self.cmd_vel_pub.publish(self.twist)

        # Show the mask and original image for debugging
        cv2.imshow("mask", mask)
        cv2.imshow("output", image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('follower')  # Initialize the ROS node
    follower = Follower()  # Create an object of Follower
    rospy.spin()  # Keep the node running

