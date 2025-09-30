from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2

client = RemoteAPIClient()
sim = client.require('sim')

sim.setStepping(True)

# PID controller variables
previous_error_x = 0
previous_error_y = 0
integral_x = 0
integral_y = 0

sim.startSimulation()
right_wheel = sim.getObject("/PioneerP3DX[1]/rightMotor")
left_wheel = sim.getObject("/PioneerP3DX[1]/leftMotor")
target_velocity = 5

while (t := sim.getSimulationTime()) < 30:
    print(f'Simulation time: {t:.2f}')
    
    sim.setJointTargetVelocity(right_wheel, target_velocity)
    sim.setJointTargetVelocity(left_wheel, target_velocity)
    camera1 = sim.getObject("/PioneerP3DX[1]/cam")

    image, resolution = sim.getVisionSensorImg(camera1)
    image = list(image)
    image = np.array(image, dtype=np.uint8)
    image = image.reshape(resolution[0], resolution[1], 3)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Apply threshold to create a binary image
    _, binary_image = cv2.threshold(image, 20, 255, cv2.THRESH_BINARY_INV)
    
    M = cv2.moments(binary_image)
    if M["m00"] != 0:
        centroid_x = int(M["m10"] / M["m00"]) 
        centroid_y = int(M["m01"] / M["m00"]) 
        # PID controller parameters for x-direction
        kp_x = 1
        ki_x = 0.01
        kd_x = 0.0

        # PID controller for x-direction
        error_x = resolution[0]/2 - centroid_x
        integral_x += error_x
        derivative_x = error_x - previous_error_x

        # PID control equation for x-direction
        steering_angle_x = kp_x * error_x + ki_x * integral_x + kd_x * derivative_x

        # PID controller parameters for y-direction
        kp_y = 1
        ki_y = 0.01
        kd_y = 0.00

        # PID controller for y-direction
        error_y = resolution[1]/2 - centroid_y
        integral_y += error_y
        derivative_y = error_y - previous_error_y

        # PID control equation for y-direction
        steering_angle_y = kp_y * error_y + ki_y * integral_y + kd_y * derivative_y

        # Combine steering adjustments in x and y directions
        combined_steering_angle = np.arctan2(steering_angle_y, steering_angle_x)
        

        # Adjust wheel velocities based on the combined_steering_angle
        left_velocity = target_velocity + combined_steering_angle
        right_velocity = target_velocity - combined_steering_angle

        # Set joint target velocities for left and right wheels
        sim.setJointTargetVelocity(right_wheel, right_velocity)
        sim.setJointTargetVelocity(left_wheel, left_velocity)

        # Update variables for the next iteration
        previous_error_x = error_x
        previous_error_y = error_y

        print(error_x)
        print(error_y)

    sim.step()

sim.stopSimulation()
