from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2

# PID controller parameters
kp = 0.1
ki = 0.01
kd = 0.05

client = RemoteAPIClient()
sim = client.require('sim')

sim.setStepping(True)

sim.startSimulation()
previous_error = 0
integral = 0

while (t := sim.getSimulationTime()) < 10:
    print(f'Simulation time: {t:.2f}')

    right_wheel = sim.getObject("/PioneerP3DX[1]/rightMotor")
    left_wheel = sim.getObject("/PioneerP3DX[1]/leftMotor")

    target_velocity = 10
    sim.setJointTargetVelocity(right_wheel, target_velocity)
    sim.setJointTargetVelocity(left_wheel, target_velocity)

    camera1 = sim.getObject("/PioneerP3DX[1]/cam1")
    image, resolution = sim.getVisionSensorImg(camera1)

    # Convert the image to a NumPy array
    image = np.array(image, dtype=np.uint8)
    image = image.reshape((resolution[0], resolution[1], 3))

    # Convert the image to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Calculate the centroid of the black path (assuming black pixels represent the path)
    _, threshold_image = cv2.threshold(gray_image, 128, 128, cv2.THRESH_BINARY)
    M = cv2.moments(threshold_image)
    centroid_x = int(M["m10"] / M["m00"]) if M["m00"] != 0 else resolution[1] // 2

    # Normalize the centroid_x to be in the range [-1, 1]
    normalized_centroid_x = (centroid_x - resolution[1] // 2) / (resolution[1] // 2)

    # PID control for steering angle
    error = normalized_centroid_x
    integral += error
    derivative = error - previous_error

    steering_angle = kp * error + ki * integral + kd * derivative

    # Update previous error for the next iteration
    previous_error = error

    # Apply steering control to set velocities for left and right wheels
    left_velocity = target_velocity - steering_angle * target_velocity
    right_velocity = target_velocity + steering_angle * target_velocity

    # Set joint target velocities
    sim.setJointTargetVelocity(right_wheel, right_velocity)
    sim.setJointTargetVelocity(left_wheel, left_velocity)

    sim.step()

sim.stopSimulation()
