import numpy as np
import cv2 

class variables:
    
    basevelocity = 5.0
    kp = 0.05
    ki = 0.01
    kd = 0.0

    integral = 0
    previous_error = 0

class image(variables) :
    
    def image_conversion(self) :
        self.image = list(self.image)
        self.image = np.array(self.image, np.uint8)
        self.image = self.image.reshape(self.resolution[0], self.resolution[1], 3)
        self.grayImage = cv2.cvtColor(self.image, cv2.COLOR_RGB2GRAY)
        _, self.binary_image = cv2.threshold(self.grayImage, 20, 225, cv2.THRESH_BINARY_INV)
        return _, self.binary_image

class pioneer(image) :

    def __init__(self, robot, client, sim) :
        self.robot = robot
        self.calculated_pid = 0.0
        self.client = client
        self.sim = sim

        self.right_wheel = sim.getObject("/PioneerP3DX["+str(self.robot)+"]/rightMotor")
        self.left_wheel = sim.getObject("/PioneerP3DX["+str(self.robot)+"]/leftMotor")
        self.camera = sim.getObject("/PioneerP3DX["+str(self.robot)+"]/Vision_sensor")

    def centroid(self) :
        self.M = cv2.moments(self.binary_image)
        self.center = 0
        if self.M["m00"] != 0:
            self.centroid_y = int(self.M["m01"] / self.M["m00"])
            self.centroid_y = int(self.M["m10"] / self.M["m00"])
        else:
            self.centroid_y = self.center
        self.error = self.center - self.centroid_y
        return self.error
    
    def pid(self, proportional):
        self.proportional = proportional
        self.error = self.proportional  * self.error
        self.integral = self.integral + self.error
        self.derivative = self.error - self.previous_error

        if (self.error):
            self.calculated_pid = self.kp * self.error + self.ki * self.integral + self.kd * self.derivative
        self.previous_error = self.error
        return self.calculated_pid

    def steering(self):
        self.steering_angle =  self.calculated_pid
        self.leftVelocity = self.basevelocity -  (self.steering_angle * self.basevelocity)
        self.rightVelocity = self.basevelocity + (self.steering_angle * self.basevelocity)
