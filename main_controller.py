from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pioneer import pioneer

# Create a RemoteAPIClient instance to connect to CoppeliaSim
client = RemoteAPIClient()
# Require the 'sim' module from the CoppeliaSim remote API
sim = client.require('sim')

# Create instances of the Pioneer class for five robots
pioneerp3dx = [pioneer(0, client, sim), pioneer(1, client, sim), pioneer(2, client, sim),
    pioneer(3, client, sim), pioneer(4, client, sim)]

# Enable stepping mode for simulation
sim.setStepping(True)
# Start the simulation in CoppeliaSim
sim.startSimulation()

# Run the simulation for a specified time (10 seconds in this case)
while (t := sim.getSimulationTime()) < 10:
    # Loop through each robot
    for robot in range(5):
        # Get image and resolution from the robot's vision sensor
        pioneerp3dx[robot].image, pioneerp3dx[robot].resolution = sim.getVisionSensorImg(pioneerp3dx[robot].camera)
        # Process the image to obtain a binary image
        _, pioneerp3dx[robot].binary_image = pioneerp3dx[robot].image_conversion()
        # Calculate the centroid and update the error
        pioneerp3dx[robot].error = pioneerp3dx[robot].centroid()
        # Adjust the steering and set joint target velocities for left and right wheels
        pioneerp3dx[robot].steering()
        sim.setJointTargetVelocity(pioneerp3dx[robot].right_wheel, pioneerp3dx[robot].basevelocity)
        sim.setJointTargetVelocity(pioneerp3dx[robot].left_wheel, pioneerp3dx[robot].basevelocity)

    # Perform a simulation step
    sim.step()

# Stop the simulation in CoppeliaSim
sim.stopSimulation()