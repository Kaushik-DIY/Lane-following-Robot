from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from robot import Robot
import constants as c


def main_task():
    """

    Set up the robot simulation environment and run the main task.

    Main task for the robot simulation. For each robot, handle the camera and steer the robot accordingly.

    Returns:
        None
    """
    # Set up the CoppeliaSim remote API
    client = RemoteAPIClient()
    sim = client.getObject("sim")
    sim = client.require("sim")
    sim.setStepping(True)          # We want to explicitly control the simulation time step
    sim.startSimulation()

    robots = []
    for i in range(c.NUM_ROBOT):
        robots.append(Robot(i, sim))
        robots[i].set_wheels_speed(-1, 1)

    while True:
        for robot in robots:
            robot.handle_camera()
        sim.step()  # Advance the simulation by one time step


if __name__ == "__main__":
    main_task()
