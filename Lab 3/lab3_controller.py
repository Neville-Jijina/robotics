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
waypoints = [(-0.154705, -0.414838), (0.325295, -0.414838), (0.325295, -0.254838)] # e.g. [(-0.1, -0.4), (0.3, -0.4), ...]
# Index indicating which waypoint the robot is reaching next
index = 0

# Get ping pong ball marker that marks the next waypoint the robot is reaching
marker = robot.getFromDef("marker").getField("translation")

def new_position(current_pos, object_pos):
    delta_x = object_pos[0] - current_pos[0]
    delta_y = object_pos[1] - current_pos[1]
    distance = np.sqrt((delta_x**2) + (delta_y**2))
    
    return distance

def bearing(current_pos, object_pos):
    delta_x = object_pos[0] - current_pos[0]
    delta_y = object_pos[1] - current_pos[1]
    target_bearing = math.atan2(delta_y, delta_x)
    
    return target_bearing
    
def heading(current_heading, bearing):
    rotation_needed = bearing - current_heading
    rotation_needed = math.atan2(np.sin(rotation_needed), np.cos(rotation_needed))
    
    return rotation_needed
    
current_state = "find_waypoint"
waypoint_found = False
new_heading = 0
new_bearing = 0
new_distance = 0
    
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
    if current_state == "find_waypoint":
        print("finding waypoint")
        new_waypoint = waypoints[index]
        waypoint_found = True
        current_state = "adjust_heading"
            # print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    
    elif current_state == "adjust_heading":
        new_bearing = bearing([pose_x, pose_y], new_waypoint)
        new_heading = heading(pose_theta, new_bearing)
        print("Need heading: ", new_heading)
        
        if(new_heading > 0.1):
            print("turn left")
            vL = -0.1 * MAX_SPEED
            vR = 0.1 * MAX_SPEED
        elif(new_heading < -0.1):
            print("turn right")
            vL = 0.1 * MAX_SPEED
            vR = -0.1 * MAX_SPEED
        else:
            print("forward")
            current_state = "forward"
        
    elif current_state == "forward":
        new_distance = new_position([pose_x, pose_y], new_waypoint)
        print(new_distance)
        
        if new_distance <= 0.03:
            print("Waypoint Found!")
            waypoint_found = False
            index = index + 1
            vL = 0 * MAX_SPEED
            vR = 0 * MAX_SPEED 
            current_state == "find_waypoint"
        else:
            vL = 0.5 * MAX_SPEED
            vR = 0.5 * MAX_SPEED 
    
    
    # print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)



