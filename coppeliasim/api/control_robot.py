import sim
import time
import sys

print("program started")
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print('connected successfully')
else:
    sys.exit('Failed to connect')

time.sleep(2)

# Define a function to set velocities for a robot
def set_robot_velocities(clientID, right_motor_handle, left_motor_handle):
    sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0.2, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0.2, sim.simx_opmode_oneshot_wait)

# Control each robot
robots = ['/PioneerP3DX[1]', '/PioneerP3DX[2]', '/PioneerP3DX[3]', '/PioneerP3DX[4]']

for i in range(1, 5):
    error_code, right_motor_handle = sim.simxGetObjectHandle(clientID, robots[i-1]+'rightmotor', sim.simx_opmode_oneshot_wait)
    error_code, left_motor_handle = sim.simxGetObjectHandle(clientID, robots[i-1]+'leftmotor', sim.simx_opmode_oneshot_wait)

    set_robot_velocities(clientID, right_motor_handle, left_motor_handle)
    time.sleep(2)  # Add a delay to allow the robot to move

# Close the connection
sim.simxFinish(clientID)
print("program ended")