import numpy as np
import cv2 

# Define a class with variables
class variables:
    # Base velocity for robot movement
    basevelocity = 5.0
    # PID controller coefficients
    kp = 0.05
    ki = 0.01
    kd = 0.0
    # Integral and previous error for PID controller
    integral = 0
    previous_error = 0

# Define a class for image processing, inheriting from the variables class
class image(variables):
    
    def image_conversion(self):
        self.image = list(self.image)
        self.image = np.array(self.image, np.uint8)
        self.image = self.image.reshape(self.resolution[0], self.resolution[1], 3)
        # Convert the image to grayscale
        self.grayImage = cv2.cvtColor(self.image, cv2.COLOR_RGB2GRAY)
        # Apply threshold to create a binary image
        _, self.binary_image = cv2.threshold(self.grayImage, 20, 225, cv2.THRESH_BINARY_INV)
        return _, self.binary_image

# Define a class for a Pioneer robot, inheriting from the image class
class pioneer(image):

    def __init__(self, robot, client, sim):
        # Initialize Pioneer object with robot index, client, and simulation object
        self.robot = robot
        self.client = client
        self.sim = sim
        self.calculated_pid = 0.0
        
        # Get handles for the robot's motors and vision sensor in CoppeliaSim
        self.right_wheel = sim.getObject("/PioneerP3DX[" + str(self.robot) + "]/rightMotor")
        self.left_wheel = sim.getObject("/PioneerP3DX[" + str(self.robot) + "]/leftMotor")
        self.camera = sim.getObject("/PioneerP3DX[" + str(self.robot) + "]/Vision_sensor")

    def centroid(self):
        # Calculate the centroid of the binary image
        self.M = cv2.moments(self.binary_image)
        self.center = 0
        if self.M["m00"] != 0:
            self.centroid_y = int(self.M["m01"] / self.M["m00"])
            self.centroid_x = int(self.M["m10"] / self.M["m00"])
        else:
            self.centroid_y = self.center
        # Calculate and return the error
        self.error = self.center - self.centroid_y
        return self.error

    def pid(self, proportional):
        # PID controller calculation
        self.proportional = proportional
        self.error = self.proportional * self.error
        self.integral = self.integral + self.error
        self.derivative = self.error - self.previous_error

        # Calculate PID output
        if self.error:
            self.calculated_pid = self.kp * self.error + self.ki * self.integral + self.kd * self.derivative
        self.previous_error = self.error
        return self.calculated_pid

    def steering(self):
        # Adjust wheel velocities based on the calculated PID output
        self.steering_angle = self.calculated_pid
        self.leftVelocity = self.basevelocity - (self.steering_angle * self.basevelocity)
        self.rightVelocity = self.basevelocity + (self.steering_angle * self.basevelocity)
