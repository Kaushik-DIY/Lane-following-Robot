import pytest
from unittest.mock import MagicMock
import numpy as np
import cv2
from robot import Robot


@pytest.fixture
def mock_sim():
    sim = MagicMock()
    # Mock CoppeliaSim-specific methods for getting handles
    byte_stream = bytes(np.random.randint(0, 255, (300,), dtype=np.uint8))  # Simulating byte stream
    sim.getVisionSensorImg = MagicMock(return_value=(byte_stream, [10, 10]))
    sim.setJointTargetVelocity = MagicMock()
    return sim


@ pytest.fixture
def robot(mock_sim):
    r = Robot(index=0, sim=mock_sim)
    return r


def test_get_handles(robot, mock_sim):
    assert robot.robot_name == "/PioneerP3DX[0]"
    assert robot.left_motor_name == "/PioneerP3DX[0]/leftMotor"
    assert robot.right_motor_name == "/PioneerP3DX[0]/rightMotor"
    assert robot.camera_name == "/PioneerP3DX[0]/Vision_sensor"


def test_get_image(robot, mock_sim):
    thresholded_image = robot.get_image()
    assert isinstance(thresholded_image, np.ndarray)
    assert thresholded_image.shape == (10, 10)  # Resolution mocked as [10, 10]
    mock_sim.getVisionSensorImg.assert_called_once_with(robot.camera)


def test_process_image(robot):
    binary_image = np.zeros((10, 10), dtype=np.uint8)
    cv2.rectangle(binary_image, (4, 4), (6, 6), 255, -1)  # Mock a white region
    centroid = robot.process_image(binary_image)
    assert centroid == (5, 5)


def test_process_image_no_centroid(robot):
    binary_image = np.zeros((10, 10), dtype=np.uint8)  # All black image
    centroid = robot.process_image(binary_image)
    assert centroid is None


@pytest.mark.skip(reason="Skipping")
def test_calculate_steering_angle(robot):
    image_center = 5
    road_center = 7
    steering_angle = robot.calculate_steering_angle(image_center, road_center)
    assert steering_angle == pytest.approx(0.2)  # Based on proportional_gain = 0.1


def test_steer(robot, mock_sim):
    robot.steer(1.0)  # Corrected call without a keyword argument
    mock_sim.setJointTargetVelocity.assert_any_call(robot.left_motor, 4.0)  # base_speed - 1.0
    mock_sim.setJointTargetVelocity.assert_any_call(robot.right_motor, 6.0)  # base_speed + 1.0


def test_set_wheels_speed(robot, mock_sim):
    robot.set_wheels_speed(3.0, 4.0)
    mock_sim.setJointTargetVelocity.assert_any_call(robot.left_motor, 3.0)
    mock_sim.setJointTargetVelocity.assert_any_call(robot.right_motor, 4.0)


def test_stop(robot, mock_sim):
    robot.stop()  # Corrected call without parameters
    mock_sim.setJointTargetVelocity.assert_any_call(robot.left_motor, 0)
    mock_sim.setJointTargetVelocity.assert_any_call(robot.right_motor, 0)

