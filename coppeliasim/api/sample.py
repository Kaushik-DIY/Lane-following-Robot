from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2 
import numpy as np
import math

class Constants:
    # Base Target velocities
    rightMotorVelocity = 5.0
    leftMotorVelocity = 5.0
    baseVelocity = 5.0

    # Gain values for P, I, D
    kp = 0.05
    ki = 0.01
    kd = 0.0
    integral = 0
    previous_error = 0

class Utils(Constants):

    def image_conversion(self, image,camera):
        image, resolution = self.sim.getVisionSensorImg(camera)
        image = list(image)
        image = np.array(image, np.uint8)
        image = image.reshape(resolution[0], resolution[1], 3)
        image = image.reshape(self.side, self.side, 3)
        gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        _, binary_image = cv2.threshold(gray_image, 20, 225, cv2.THRESH_BINARY)
        return _, binary_image

class PioneerP3DX(Utils):

    def __init__(self, i, client, sim):
        self.i = i
        self.client = client
        self.sim = sim
        self.right_wheel = sim.getObject(f"/PioneerP3DX[{self.i}]/rightMotor")
        self.left_wheel = sim.getObject(f"/PioneerP3DX[{self.i}]/leftMotor")
        self.camera = sim.getObject(f"/PioneerP3DX[{self.i}]/cam")

    def centroids(self):
        M = cv2.moments(self.thresholded_image)
        if M["m00"] != 0:
            centroid_y = int(M["m01"] / M["m00"])
            centroid_x = int(M["m10"] / M["m00"])
        else:
            centroid_y = self.center
        error = self.center - centroid_y
        return error

    def PID(self, multiplier):
        # PID controller
        self.multiplier = multiplier
        self.error = self.multiplier * self.error
        self.integral = self.integral + self.error
        self.derivative = self.error - self.error_prev

        # Calculate PID output
        if self.error:
            pid_value = self.kp * self.error + self.ki * self.integral + self.kd * self.derivative
        else:
            pid_value = 0.0

        # Save current error for the next iteration
        self.error_prev = self.error
        return pid_value

    def steering(self):
        steering_angle = self.scale_factor * self.pid_value
        self.targetVelocity_l = self.baseVelocity - (steering_angle / 180) * self.baseVelocity
        self.targetVelocity_r = self.baseVelocity + (steering_angle / 180) * self.baseVelocity

if __name__ == "__main__":
    

    client = RemoteAPIClient()
    sim = client.require('sim')

    Pioneer = [PioneerP3DX(0, client, sim), PioneerP3DX(1, client, sim), PioneerP3DX(2, client, sim), 
    PioneerP3DX(3, client, sim), PioneerP3DX(4, client, sim) ]
    sim.setStepping(True)
    sim.startSimulation()

    while (t := sim.getSimulationTime()) < 15:
        print(f'Simulation time: {t:.2f}')
        for Pioneer in Pioneer:
            Pioneer.image, Pioneer.resolution = sim.getVisionSensorImg(Pioneer.camera)
            Pioneer.threshold, Pioneer.thresholded_image = Pioneer.image_conversion(Pioneer.image)
            Pioneer.error = Pioneer.centroids()
            multiplier = -1 if Pioneer.i == 1 or Pioneer.i == 2 else 1
            Pioneer.pid_value = Pioneer.PID(multiplier)
            Pioneer.steering()
            sim.setJointTargetVelocity(Pioneer.right_wheel, Pioneer.baseVelocity)
            sim.setJointTargetVelocity(Pioneer.left_wheel, Pioneer.baseVelocity)

        sim.step()

    sim.stopSimulation()
