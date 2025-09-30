import sim
from time import sleep as delay
import numpy as np
import cv2
import sys

def setup_robot(clientID, robot_name):
    errorCode, left_motor_handle = sim.simxGetObjectHandle(
        clientID, f'{robot_name}_leftMotor', sim.simx_opmode_oneshot_wait)
    errorCode, right_motor_handle = sim.simxGetObjectHandle(
        clientID, f'{robot_name}_rightMotor', sim.simx_opmode_oneshot_wait)

    errorCode, camera_handle = sim.simxGetObjectHandle(
        clientID, f'{robot_name}_cam', sim.simx_opmode_oneshot_wait)

    return left_motor_handle, right_motor_handle, camera_handle

def move_robot(clientID, left_motor_handle, right_motor_handle, lSpeed, rSpeed):
    errorCode = sim.simxSetJointTargetVelocity(
        clientID, left_motor_handle, lSpeed, sim.simx_opmode_streaming)
    errorCode = sim.simxSetJointTargetVelocity(
        clientID, right_motor_handle, rSpeed, sim.simx_opmode_streaming)

def read_vision_sensor(clientID, camera_handle):
    returnCode, resolution, image = sim.simxGetVisionSensorImage(
        clientID, camera_handle, 0, sim.simx_opmode_buffer)

    im = np.array(image, dtype=np.uint8)
    im.resize([resolution[1], resolution[0], 3])

    im = cv2.flip(im, 0)
    im = cv2.rotate(im, cv2.ROTATE_90_COUNTERCLOCKWISE)
    im = cv2.resize(im, (256, 256))
    im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)

    return im

print('Program started')
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if (clientID != -1):
    print('Connected to remote API server')

else:
    sys.exit('Failed connecting to remote API server')

delay(1)

# Setup four robots
robot_names = ['/Pioneer3DX[1]/', '/Pioneer3DX[2]/', '/Pioneer3DX[3]/', '/Pioneer3DX[4]/']
robots = [setup_robot(clientID, name) for name in robot_names]

try:
    while (1):
        for i, robot in enumerate(robots):
            left_motor_handle, right_motor_handle, camera_handle = robot

            # Read the vision sensor for each robot
            im = read_vision_sensor(clientID, camera_handle)

            # Display the image for each robot
            cv2.imshow(f"Robot {i+1}", im)

        com = cv2.waitKey(1)
        if (com == ord('q')):
            break
        elif (com == ord('w')):
            for robot in robots:
                move_robot(clientID, robot[0], robot[1], 0.2, 0.2)
        elif (com == ord('a')):
            for robot in robots:
                move_robot(clientID, robot[0], robot[1], -0.1, 0.2)
        elif (com == ord('d')):
            for robot in robots:
                move_robot(clientID, robot[0], robot[1], 0.2, -0.1)
        elif (com == ord('s')):
            for robot in robots:
                move_robot(clientID, robot[0], robot[1], -0.2, -0.2)
        else:
            for robot in robots:
                move_robot(clientID, robot[0], robot[1], 0, 0)
        com = 'o'

    cv2.destroyAllWindows()
except:
    cv2.destroyAllWindows()
