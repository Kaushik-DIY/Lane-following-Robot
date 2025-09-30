import numpy as np
import cv2 as cv
from utils import utils

class PioneerP3DX(utils) :

    def __init__(self, i, client, sim) :
        self.i = i
        self.client = client
        self.sim = sim
        self.right_wheel = sim.getObject("/PioneerP3DX["+str(self.i)+"]/rightMotor")
        self.left_wheel = sim.getObject("/PioneerP3DX["+str(self.i)+"]/leftMotor")
        self.camera = sim.getObject("/PioneerP3DX["+str(self.i)+"]/cam")
        
    def centroids(self) :
        self.M = cv.moments(self.thresholded_image)
        if self.M["m00"] != 0:
            self.centroid_y = int(self.M["m01"] / self.M["m00"])
            self.centroid_x = int(self.M["m10"] / self.M["m00"]) 
        else:
            self.centroid_y = self.center
        self.error = self.center - self.centroid_y
        return self.error
    
    def PID(self, multiplier):
        self.multiplier = multiplier
        self.error = self.multiplier  * self.error
        self.integral = self.integral + self.error
        self.derivative = self.error - self.error_prev

        # Calculate PID output
        if (self.error):
            self.pid_value = self.kp * self.error + self.ki * self.integral + self.kd * self.derivative
        else:
            self.pid_value = 0.0

        # Save current error for the next iteration
        self.error_prev = self.error
        return self.pid_value

    def steering(self):
        # Function to vary speeds of left and right motors differentially
        # to introduce differential steering
        self.steering_angle = self.scale_factor * self.pid_value
        self.targetVelocity_l = self.speed - (self.steering_angle / 180) * self.speed
        self.targetVelocity_r = self.speed + (self.steering_angle / 180) * self.speed
