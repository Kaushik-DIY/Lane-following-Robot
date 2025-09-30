import sim
import time
import sys
import numpy as np

# Function to check if the robot is on a black path based on the captured image
def check_black_path(image, resolution):
    if not image:
        return False

    # Extract image data
    image_data = image[2]
    image_array = np.array(image_data, dtype=np.uint8)
    image_array = image_array.reshape([resolution[1], resolution[0], 3], order='F')

    # Check if the center pixel is black
    center_pixel_color = image_array[resolution[1] // 2, resolution[0] // 2, :]
    return np.all(center_pixel_color == [0, 0, 0])

# Function to control each robot
def control_robot(client_id, left_motor, right_motor, front_sensor, camera, left_velocity, right_velocity):
    _, _, _, _, _ = sim.simxReadProximitySensor(client_id, front_sensor, sim.simx_opmode_streaming)
    _, resolution, image = sim.simxGetVisionSensorImage(client_id, camera, 1, sim.simx_opmode_streaming)

    for _ in range(200):
        _, _, detected_point, _, _ = sim.simxReadProximitySensor(client_id, front_sensor, sim.simx_opmode_buffer)
        _, resolution, image = sim.simxGetVisionSensorImage(client_id, camera, 1, sim.simx_opmode_buffer)

        # Check if the robot is on a black path
        is_on_black_path = check_black_path(image, resolution)

        if is_on_black_path:
            print(f'Robot {client_id} - On Black Path - Steering')
            sim.simxSetJointTargetVelocity(client_id, right_motor, right_velocity, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, left_motor, left_velocity, sim.simx_opmode_blocking)
        else:
            print(f'Robot {client_id} - Not on Black Path - Turn')
            for _ in range(10):
                sim.simxSetJointTargetVelocity(client_id, right_motor, -right_velocity, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(client_id, left_motor, left_velocity, sim.simx_opmode_blocking)

        time.sleep(0.1)

print("Program started")

# Connect to the simulation
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print('Connected successfully')
else:
    sys.exit('Failed to connect')

time.sleep(2)

# Define parameters for each robot
robot_params = [
    {'robot_name': '/PioneerP3DX[0]', 'robot_number': 1, 'left_velocity': 0.2, 'right_velocity': 0.2, 'vision_sensor_name': 'cam0'},
    {'robot_name': '/PioneerP3DX[1]', 'robot_number': 2, 'left_velocity': 0.2, 'right_velocity': 0.2, 'vision_sensor_name': 'cam1'},
    {'robot_name': '/PioneerP3DX[2]', 'robot_number': 3, 'left_velocity': 0.2, 'right_velocity': 0.2, 'vision_sensor_name': 'cam2'},
    {'robot_name': '/PioneerP3DX[3]', 'robot_number': 4, 'left_velocity': 0.2, 'right_velocity': 0.2, 'vision_sensor_name': 'cam3'},
    {'robot_name': '/PioneerP3DX[4]', 'robot_number': 5, 'left_velocity': 0.2, 'right_velocity': 0.2, 'vision_sensor_name': 'cam4'},
]

# Control each robot
for params in robot_params:
    robot_name = params['robot_name']
    robot_number = params['robot_number']
    vision_sensor_name = params['vision_sensor_name']

    # Get handles for motors and sensors
    error_code, right_motor_handle = sim.simxGetObjectHandle(clientID, f'{robot_name}_rightMotor#{robot_number}', sim.simx_opmode_oneshot_wait)
    error_code, left_motor_handle = sim.simxGetObjectHandle(clientID, f'{robot_name}_leftMotor#{robot_number}', sim.simx_opmode_oneshot_wait)
    error_code, camera_handle = sim.simxGetObjectHandle(clientID, f'{robot_name}_{vision_sensor_name}#{robot_number}', sim.simx_opmode_oneshot_wait)

    # Control the robot
    control_robot(clientID, left_motor_handle, right_motor_handle, front_sensor_handle, camera_handle,
                  params['left_velocity'], params['right_velocity'])

    time.sleep(2)  # Add a delay to allow the robot to move

# Function to set velocities for a robot
def set_robot_velocities(clientID, right_motor_handle, left_motor_handle):
    sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0.2, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0.2, sim.simx_opmode_oneshot_wait)

# Control each robot
robot_names = ['/Pioneer_p3dx[0]', '/Pioneer_p3dx[1]', '/Pioneer_p3dx[2]', '/Pioneer_p3dx[3]', '/Pioneer_p3dx[4]']

for robot_name in robot_names:
    error_code, right_motor_handle = sim.simxGetObjectHandle(clientID, f'{robot_name}/rightMotor', sim.simx_opmode_oneshot_wait)
    error_code, left_motor_handle = sim.simxGetObjectHandle(clientID, f'{robot_name}/leftMotor', sim.simx_opmode_oneshot_wait)

    set_robot_velocities(clientID, right_motor_handle, left_motor_handle)
    time.sleep(2)  # Add a delay to allow the robot to move

# Close the connection
sim.simxFinish(clientID)
print("Program ended")