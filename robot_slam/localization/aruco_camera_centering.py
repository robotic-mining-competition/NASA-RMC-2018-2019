#!/usr/bin/env python

# Receives ROS messages about if an Aruco marker has been detected and the average
# x location of its corners. Centers the camera using a P feedback loop on this x-coordinate.

# Import ROS packages
import rospy
import rospkg
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Int32
from cv_bridge import CvBridge
import tf.transformations

# Import other required packages
import cv2.aruco as aruco
import numpy as np


class ArucoCamera:

    def __init__(self):

        # Initialize an Aruco camera object that contains the camera's current x-coord,
        # the value it is trying to reach, and parameters defining whether or not an Aruco marker 
        # is in the frame.

        # Servo equivalent x value in center of frame; ROS Int32 Object
        # Use self.theta.data to access angle 
        self.theta = Int32()   
        self.setPoint = 0
        self.markerCenter = 0 # Stores the center position of the marker
        self.minTheta = -90
        self.maxTheta = 90
        self.markerDetected = False
        self.sweepDirection = 1
        self.k = .05 # Proportional gain
        self.delta_theta = 2 

        self.servoThetaPub = rospy.Publisher("aruco/servo_theta", Int32, queue_size=0)

    # Decides whether to track Aruco marker or sweep camera
    def updateMarkerDetected(self,detected):
        self.markerDetected = detected  

    # Update the x position of the Aruco marker
    def updateSetPoint(self,markerCenter):
        self.markerCenter = markerCenter
    
    # Rotate servo to track the Aruco marker or sweep the camera to find a marker
    def updatePosition(self):
        if self.markerDetected:
            self.setPoint = self.markerCenter #servo will track Aruco marker
        else:
            
            # Update set point
            self.setPoint += self.delta_theta*self.sweepDirection

        # Check if the set point has passed servo bounds
        if (self.setPoint >= self.maxTheta):
            self.setPoint = self.maxTheta
            self.sweepDirection = -1

        else if self.setPoint <= self.minTheta:
            self.setPoint = self.minTheta
            self.sweepDirection = 1

        # Update servo orientation
        self.theta.data += self.k * (self.setPoint - self.theta.data)
        self.servoThetaPub.publish(self.theta)


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node('aruco_camera')

    # Initialize ArucoCamera object
    arucoCamera = ArucoCamera()

    # Create subscribers for detecting an Aruco board and obtaining the avg x-val of corners
    markerDetectedSub = rospy.Subscriber("aruco/marker_detected", Bool, arucoCamera.updateMarkerDetected)
    avgCornerSub = rospy.Subscriber("aruco/avg_of_corners", Int32, arucoCamera.updateSetPoint)

    # Set loop rate
    r = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        arucoCamera.updatePosition()
        r.sleep()
    