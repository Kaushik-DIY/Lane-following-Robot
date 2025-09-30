from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2

client = RemoteAPIClient()
sim = client.require('sim')

sim.setStepping(True)

# PID controller variables
previous_error = [0] * 5
integral = [0] * 5

sim.startSimulation()

while (t := sim.getSimulationTime()) < 30:
    print(f'Simulation time: {t:.2f}')

    for i in range(0, 5):
        right_wheel = sim.getObject(f"/PioneerP3DX[{i}]/rightMotor")
        left_wheel = sim.getObject(f"/PioneerP3DX[{i}]/leftMotor")
        target_velocity = 8

        sim.setJointTargetVelocity(right_wheel, target_velocity)
        sim.setJointTargetVelocity(left_wheel, target_velocity)

        camera = sim.getObject(f"/PioneerP3DX[{i}]/cam")
        image, resolution = sim.getVisionSensorImg(camera)

        image = list(image)
        image = np.array(image, dtype=np.uint8)
        image = image.reshape(resolution[0], resolution[1], 3)
        gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        # Apply threshold to create a binary image
        _, binary_image = cv2.threshold(gray_image, 20, 255, cv2.THRESH_BINARY)
        
        # Calculate the centroid
        M = cv2.moments(binary_image)
        if M["m00"] != 0:
            centroid_x = int(M["m10"] / M["m00"]) 
            centroid_y = int(M["m01"] / M["m00"]) 
            # PID controller parameters
            kp = 0.01
            ki = 0.0
            kd = 0.01
            # PID controller
            error = resolution[0]/2 - centroid_x
            integral[i] += error
            derivative = error - previous_error[i]

            # PID control equation
            steering_angle = kp * error + ki * integral[i] + kd * derivative

            # Calculate individual wheel velocities based on the steering angle
            left_velocity = target_velocity - steering_angle
            right_velocity = target_velocity + steering_angle

            # Set joint target velocities for left and right wheels
            sim.setJointTargetVelocity(right_wheel, right_velocity)
            sim.setJointTargetVelocity(left_wheel, left_velocity)

            # Update variables for the next iteration
            previous_error[i] = error

    sim.step()
sim.stopSimulation()
