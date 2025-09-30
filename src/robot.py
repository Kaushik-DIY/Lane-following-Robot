import cv2
import numpy as np


class Robot:
    """Robot class"""

    def __init__(self, index, sim, base_speed=5):
        self.robot_name = f"/PioneerP3DX[{index}]"
        self.left_motor_name = f"{self.robot_name}/leftMotor"
        self.right_motor_name = f"{self.robot_name}/rightMotor"
        self.camera_name = f"{self.robot_name}/Vision_sensor"
        self.base_speed = base_speed
        self.left_speed = 0
        self.right_speed = 0
        self.sim = sim
        self.get_handles()

    def get_handles(self):
        """Get handles for robot components

        .. todo:: Implement the function to get handles for various robot components

        Returns:
            None
        """

    def get_image(self,):
        """
        Get RGB image from the camera, use opencv to convert it to grayscale and threshold it and return the thresholded image.

        .. todo:: Implement the function to get image from the camera and threshold it

        Returns:
            thresholded_image (numpy.ndarray): Thresholded image
        """

    def handle_camera(self,):
        """
        Entry point for the robot to handle the camera. Read the image from camera, calculate the centroid of the road and steer the robot accordingly.

        .. todo:: Implement the main logic for the robot to handle the camera and steer the robot accordingly

        Returns:
            None
        """

    def process_image(self, image):
        """
        Process the image to find the centroid of the road.

        .. todo:: Implement the function to find the centroid of the road in the grayscale image

        Returns:
         cx, cy (int, int): Centroid of the road
        """

    def calculate_steering_angle(self, image_center, road_center):
        """
        Calculate the steering angle based on the deviation of the road centroid from the image center.

        .. todo:: Implement the function to calculate the steering angle based on the deviation of the road centroid from the image center

        Parameters:
            image_center (int): Center of the image
            road_center (int): Centroid of the road
        Returns:
            steering_angle (float): Steering angle
        """

    def steer(self, steering=0):
        """
        Steer the robot based on the steering angle by adjusting the speed of the left and right motors.

        .. todo:: Implement the function to steer the robot.

        Parameters:
         steering (float): Steering 
        Returns:
            None
        """

    def set_wheels_speed(self, left_speed, right_speed):
        """
        Set the speed of the left and right motors of the robot.

        .. todo:: Implement the function to set the speed of the left and right motors of the robot by calling the CoppeliaSim remote API.

        Parameters:
            left_speed (float): Speed of the left motor
            right_speed (float): Speed of the right motor
        Returns:
         None
        """

    def stop(self,):
        """
        Stop the robot by setting the speed of the left and right motors to zero.

        .. todo:: Implement the function to stop the robot by setting the speed of the left and right motors to zero.

        Returns:
            None
        """
