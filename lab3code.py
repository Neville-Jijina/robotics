"""csci3302_lab3 controller."""

# You may need to import some classes of the controller module.
import math
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np

pose_x = 0
pose_y = 0
pose_theta = 0

# create the Robot instance.
robot = Supervisor()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1257 # ePuck wheel speed in m/s
MAX_SPEED = 6.28

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10): robot.step(SIM_TIMESTEP)  

vL = 0
vR = 0

# Initialize gps and compass for odometry
gps = robot.getDevice("gps")
gps.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)

# TODO: Find waypoints to navigate around the arena while avoiding obstacles
# Use shift+drag on the ping pong marker in the simulator to find good waypoints.
# Add them as (x, y) tuples. You need at least one waypoint before running!
waypoints = [(-0.154705, -0.414838, 0.02), (0.325295, -0.414838, 0.02), (0.325295, -0.254838, 0.02)] # e.g. [(-0.1, -0.4), (0.3, -0.4), ...]
# Index indicating which waypoint the robot is reaching next
index = 0

# Get ping pong ball marker that marks the next waypoint the robot is reaching
marker = robot.getFromDef("marker").getField("translation")

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:
    # Safety check: make sure waypoints are defined
    if len(waypoints) == 0:
        print("ERROR: No waypoints defined! Please add waypoints to the waypoints list.")
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        continue

    # Set the position of the marker
    marker.setSFVec3f([waypoints[index][0], waypoints[index][1], 0.01])
    
    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    # Read pose_x, pose_y, pose_theta from gps and compass
    pose_x = gps.getValues()[0]
    pose_y = gps.getValues()[1]
    pose_theta = np.arctan2(compass.getValues()[0], compass.getValues()[1])
    
    # TODO: controller
    # part 2:
    #position error 
    rho = np.sqrt((goal_x - pose_x)**2 + (goal_y - pose_y)**2)
    #bearing error 
    alpha = np.arctan2(goal_y - pose_y, goal_x - pose_x) - pose_theta
    #heading error
    eta = goal_theta - pose_theta
    #wrap alpha and eta from -pi to pi 
    alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
    eta = np.arctan2(np.sin(eta), np.cos(eta))
    
    #part 3: 
    # controller gains 
    #will need tuning 
    p1 = 1.0 #distance
    p2 = 1.0 #bearing
    p3 = 1.0 #heading 
    
    #desired translational velocity 
    x_dot_R = p1 * rho
    
    #desired rotational velocity 
    theta_dot = p2 * alpha + p3 * eta
    
    #distance between left & right wheels 
    d = EPUCK_AXLE_DIAMETER
    
    #v = r*w, so solve for r  
    r = EPUCK_MAX_WHEEL_SPEED / MAX_SPEED
    
    #calculate left and right angular velocities 
    phi_dot_L = (x_dot_R - (theta_dot * d / 2)) / r
    phi_dot_R = (x_dot_R + (theta_dot * d / 2)) / r
    
    #make sure epuck motors aren't above max speed or below -max speed
    vL = np.clip(phi_dot_L, -MAX_SPEED, MAX_SPEED)
    vR = np.clip(phi_dot_R, -MAX_SPEED, MAX_SPEED)
    
    #if robot is within 2cm of waypoint, waypoint is reached 
    if rho < 0.02
       # check if this is last waypoint 
       if index == len(waypoints) - 1:
           # stop robot 
           vL = 0
           vR = 0
       else: 
           # go to next waypoint if not last waypoint 
           index += 1


    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
