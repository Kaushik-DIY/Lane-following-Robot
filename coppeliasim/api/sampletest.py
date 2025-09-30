from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2

client = RemoteAPIClient()
sim = client.require('sim')

sim.setStepping(True)

# Threshold value for grass detection
grass_threshold = 20

# Turning behavior variables
turning = False
turn_direction = 1  # 1 for right turn, -1 for left turn

# PID controller variables
previous_error = 0
integral = 0

sim.startSimulation()

while (t := sim.getSimulationTime()) < 30:
    print(f'Simulation time: {t:.2f}')
    right_wheel = sim.getObject("/PioneerP3DX[1]/rightMotor")
    left_wheel = sim.getObject("/PioneerP3DX[1]/leftMotor")

    target_velocity = 5

    sim.setJointTargetVelocity(right_wheel, target_velocity)
    sim.setJointTargetVelocity(left_wheel, target_velocity)

    camera1 = sim.getObject("/PioneerP3DX[1]/cam")
    image, resolution = sim.getVisionSensorImg(camera1)
    image = list(image)
    image = np.array(image, dtype=np.uint8)
    image = image.reshape(resolution[0], resolution[1], 3)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Apply threshold to create a binary image
    _, binary_image = cv2.threshold(image, grass_threshold, 255, cv2.THRESH_BINARY_INV)
    
    M = cv2.moments(binary_image)
    if M["m00"] != 0:
        centroid_x = int(M["m10"] / M["m00"]) 
        centroid_y = int(M["m01"] / M["m00"]) 

        if centroid_x < resolution[1] // 2:
            turn_direction = 1  # Turn right
        else:
            turn_direction = -1  # Turn left

        turning = True
    else:
        turning = False

    # If turning, adjust the velocities
    if turning:
        right_velocity = target_velocity * turn_direction
        left_velocity = -target_velocity * turn_direction
    else:
        # PID controller parameters
        kp = 0.01
        ki = 0.0
        kd = 0.0

        # PID controller
        error = resolution[0] / 2 - centroid_x
        integral += error
        derivative = error - previous_error

        # PID control equation
        steering_angle = kp * error + ki * integral + kd * derivative

        left_velocity = target_velocity + steering_angle
        right_velocity = target_velocity - steering_angle

        # Update variables for the next iteration
        previous_error = error

    # Set joint target velocities for left and right wheels
    sim.setJointTargetVelocity(right_wheel, right_velocity)
    sim.setJointTargetVelocity(left_wheel, left_velocity)

    sim.step()

sim.stopSimulation()
