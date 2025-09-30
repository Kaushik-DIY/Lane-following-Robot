from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2

class PioneerController:
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.sim.setStepping(True)

        # PID controller parameters
        self.kp = 0.01
        self.ki = 0.0
        self.kd = 0.00

        # PID controller variables
        self.previous_error = [0] * 5
        self.integral = [0] * 5

    def setup_simulation(self):
        self.sim.startSimulation()

    def get_image(self, index):
        camera = self.sim.getObject(f"/PioneerP3DX[{index}]/cam")
        image, resolution = self.sim.getVisionSensorImg(camera)

        image = list(image)
        image = np.array(image, dtype=np.uint8)
        image = image.reshape(resolution[0], resolution[1], 3)
        gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        return gray_image, resolution

    def apply_threshold(self, image):
        _, binary_image = cv2.threshold(image, 20, 255, cv2.THRESH_BINARY_INV)
        return binary_image

    def calculate_centroid(self, binary_image):
        M = cv2.moments(binary_image)
        if M["m00"] != 0:
            centroid_x = int(M["m10"] / M["m00"]) 
            centroid_y = int(M["m01"] / M["m00"]) 
            return centroid_x, centroid_y
        return None

    def pid_controller(self, index, target_velocity, centroid_x):
        error = self.sim.getVisionSensorResolution(camera)[0] / 2 - centroid_x
        self.integral[index] += error
        derivative = error - self.previous_error[index]

        steering_angle = self.kp * error + self.ki * self.integral[index] + self.kd * derivative
        return steering_angle

    def set_wheel_velocities(self, index, target_velocity, steering_angle):
        right_wheel = self.sim.getObject(f"/PioneerP3DX[{index}]/rightMotor")
        left_wheel = self.sim.getObject(f"/PioneerP3DX[{index}]/leftMotor")

        left_velocity = target_velocity - steering_angle
        right_velocity = target_velocity + steering_angle

        self.sim.setJointTargetVelocity(right_wheel, right_velocity)
        self.sim.setJointTargetVelocity(left_wheel, left_velocity)

    def run_simulation(self, duration=30):
        while (t := self.sim.getSimulationTime()) < duration:
            print(f'Simulation time: {t:.2f}')

            for i in range(0, 5):
                target_velocity = 5

                gray_image, resolution = self.get_image(i)
                binary_image = self.apply_threshold(gray_image)
                centroid = self.calculate_centroid(binary_image)

                if centroid is not None:
                    centroid_x, _ = centroid
                    steering_angle = self.pid_controller(i, target_velocity, centroid_x)
                    self.set_wheel_velocities(i, target_velocity, steering_angle)

            self.sim.step()

        self.sim.stopSimulation()
if __name__ == "__main__":
    pioneer_controller = PioneerController()
    pioneer_controller.setup_simulation()
    pioneer_controller.run_simulation()