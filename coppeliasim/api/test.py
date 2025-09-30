from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2

client = RemoteAPIClient()
sim = client.require('sim')

sim.setStepping(True)

# PID controller parameters
kp = 1.0
ki = 0.0
kd = 0.0

# PID controller variables
previous_error = [0] * 5
integral = [0] * 5

# Setpoint for the PID controller
setpoint = 0.0

# Threshold for black color in HSV space
lower_black = np.array([0, 0, 0], dtype=np.uint8)
upper_black = np.array([128, 128, 30], dtype=np.uint8)

sim.startSimulation()

while (t := sim.getSimulationTime()) < 3:
    print(f'Simulation time: {t:.2f}')

    for i in range(0, 5):
        right_wheel = sim.getObject("/PioneerP3DX[{i}]/rightMotor")
        left_wheel = sim.getObject("/PioneerP3DX[{i}]/leftMotor")
        target_velocity = 20

        sim.setJointTargetVelocity(right_wheel, target_velocity)
        sim.setJointTargetVelocity(left_wheel, target_velocity)

        camera = sim.getObject("/PioneerP3DX[{i}]/cam")
        image, resolution = sim.getVisionSensorImg(camera)

        image = list(image)
        image = np.array(image, dtype=np.uint8)
        image = image.reshape(resolution[0], resolution[1], 3)
        
        # Convert image to HSV for better color processing
        hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        # Threshold to extract black regions
        black_mask = cv2.inRange(hsv_image, lower_black, upper_black)

        # Bitwise AND to get black regions
        black_result = cv2.bitwise_and(image, image, mask=black_mask)

        # Convert to grayscale
        gray_image = cv2.cvtColor(black_result, cv2.COLOR_RGB2GRAY)

        # Apply threshold to create a binary image
        _, binary_image = cv2.threshold(gray_image, 128, 128, cv2.THRESH_BINARY)

        # Find contours in the binary image
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            # Calculate the centroid of the largest contour
            M = cv2.moments(largest_contour)
            centroid_x = int(M["m10"] / (M["m00"] + 1e-5)) if M["m00"] != 0 else resolution[1] // 2

            # Normalize the centroid_x to be in the range [-1, 1]
            normalized_centroid_x = (2 * centroid_x / resolution[1]) - 1 if resolution[1] != 0 else 0

            # PID controller
            error = setpoint - normalized_centroid_x
            integral[i-1] += error
            derivative = error - previous_error[i-1]

            # PID control equation
            steering_angle = kp * error + ki * integral[i-1] + kd * derivative

            # Example: Adjust the velocity of left and right wheels based on steering angle
            # Calculate individual wheel velocities based on the steering angle
            left_velocity = target_velocity - (steering_angle * target_velocity)
            right_velocity = target_velocity + (steering_angle * target_velocity)

            # Set joint target velocities for left and right wheels
            sim.setJointTargetVelocity(right_wheel, right_velocity)
            sim.setJointTargetVelocity(left_wheel, left_velocity)

            # Update variables for the next iteration
            previous_error[i-1] = error

        sim.step()

sim.stopSimulation()
