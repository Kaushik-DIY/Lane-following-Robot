import sim
import time
import sys
import numpy as np
import cv2

class LineTracerRobot:

    def __init__(self, sim, clientID, robot_name):
        self._sim = sim
        self.clientID = clientID
        self.robot_name = robot_name
        self._initialize_robot()

    def _initialize_robot(self):
        _, self.leftMotor = sim.simxGetObjectHandle(self.clientID, f"{self.robot_name}DynamicLeftJoint", sim.simx_opmode_oneshot_wait)
        _, self.rightMotor = sim.simxGetObjectHandle(self.clientID, f"{self.robot_name}DynamicRightJoint", sim.simx_opmode_oneshot_wait)
        _, self.visionSensor = sim.simxGetObjectHandle(self.clientID, f"{self.robot_name}VisionSensor", sim.simx_opmode_oneshot_wait)

    def _set_two_motor(self, left: float, right: float):
        self._sim.simxSetJointTargetVelocity(self.clientID, self.leftMotor, left, sim.simx_opmode_streaming)
        self._sim.simxSetJointTargetVelocity(self.clientID, self.rightMotor, right, sim.simx_opmode_streaming)

    def read_vision_sensor(self):
        _, resolution, image = self._sim.simxGetVisionSensorImage(self.clientID, self.visionSensor, 0, sim.simx_opmode_oneshot_wait)
        img = np.array(image, dtype=np.uint8)
        img.resize([resolution[1], resolution[0], 3])
        return img

    def rotate_right(self, speed=2.0):
        self._set_two_motor(speed, -speed)

    def rotate_left(self, speed=2.0):
        self._set_two_motor(-speed, speed)

    def move_forward(self, speed=2.0):
        self._set_two_motor(speed, speed)

    def move_backward(self, speed=2.0):
        self._set_two_motor(-speed, -speed)

    def stop(self):
        self._set_two_motor(0.0, 0.0)

# Constants
TIMEOUT_LIMIT = 60
SLEEP_TIME = 0.2

print('Program started')
sim.simxFinish(-1)  # Close all opened connections

# Connect to CoppeliaSim
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print('Connected to remote API server')

    # Create robot instances
    robot_names = ['Robot1', 'Robot2', 'Robot3', 'Robot4']
    robots = [LineTracerRobot(sim, clientID, name) for name in robot_names]

    startTime = time.time()

    while time.time() - startTime < TIMEOUT_LIMIT:
        for robot in robots:
            # Read the vision sensor:
            vision_data = robot.read_vision_sensor()

            # Perform image processing or analysis based on your requirements
            # For simplicity, let's check the average intensity of the image
            average_intensity = np.mean(vision_data)

            print(f"{robot.robot_name}: Average Intensity: {average_intensity}")

            # Add your logic for robot control based on vision sensor data here

        time.sleep(SLEEP_TIME)

    # Stop all robots
    for robot in robots:
        robot.stop()
        time.sleep(0.5)  # Delay to execute the command

    print('Program ended')

    # Close all opened connections
    sim.simxFinish(-1)
else:
    print('Connection failed!!')
    sys.exit('Could not connect')


