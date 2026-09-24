"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module.
import math
from controller import Robot, Motor, DistanceSensor
# import os

# Ground Sensor Measurements under this threshold are black
# measurements above this threshold can be considered white.
# TODO: Set a reasonable threshold that separates "line detected" from "no line detected"
GROUND_SENSOR_THRESHOLD = 600

# These are your pose values that you will update by solving the odometry equations
pose_x = 0
pose_y = 0
pose_theta = 0

# Index into ground_sensors and ground_sensor_readings for each of the 3 onboard sensors.
LEFT_IDX = 0
CENTER_IDX = 1
RIGHT_IDX = 2

# create the Robot instance.
robot = Robot()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053  # ePuck's wheels are 53mm apart.
# TODO: set the ePuck wheel speed in m/s after measuring the speed (Part 1)
EPUCK_MAX_WHEEL_SPEED = 0
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
#gs0- left
#gs1- center
#gs2- right 
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice(
    'gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10):
    robot.step(SIM_TIMESTEP)

# Initialize variable for left and right speed
vL = 0
vR = 0

#set up current state 
currentState = "speed_measurement"

#speed measurement 
EPUCK_MAX_WHEEL_SPEED = 0.11 #m/s

#general global variables 

startLineTime = 0

def update_odometry(x,y,theta,vL,vR):
   
   delta_time = SIM_TIMESTEP / 1000
   
   left = (vL / MAX_SPEED) * EPUCK_MAX_WHEEL_SPEED
   right = (vR / MAX_SPEED) * EPUCK_MAX_WHEEL_SPEED
   
   forward_speed = (left + right) / 2
   angle_speed = (right - left) / EPUCK_AXLE_DIAMETER
   
   x += forward_speed * math.cos(theta) * delta_time
   y += forward_speed * math.sin(theta) * delta_time
   theta += angle_speed * delta_time 
   
   return x,y,theta


atStartLine = False
startTiming = 0.0
onStartLine = False

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:

    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    # TODO: Uncomment to see the ground sensor values!
    # TODO: But when you don't need it, please comment it so you have a clean terminal.
    #print(gsr) #when a sensor detects line the range is from 297-303 ish 

    # Part 1
    # TODO: Implement Maximum Speed Measurement under state "speed_measurement"
    # TODO: Save the speed within XZ-plane to EPUCK_MAX_WHEEL_SPEED after measuring it.
    if currentState == "speed_measurement":
        vL = MAX_SPEED
        vR = MAX_SPEED
        #have the robot go max speed until line is detected
        #line is detected when all three sensors detect from 297-303 ish range 
        if gsr[0] > 297 and gsr[0] < 303 and gsr[1] > 297 and gsr[1] < 303 and gsr[2] > 297 and gsr[2] < 303:
            vL = 0
            vR = 0 
            #print(robot.getTime())
            #print (SIM_TIMESTEP)
            #linear translation is -0.2-0.190295 
            #time step was 3.456 s
            # speed = 0.11 m/s 
            currentState = "line_follower"
           
    # Part 2
    # TODO: Implement Line Following under state "line_follower"
    # TODO: Also implement update_odometry and then call update_odometry here
    # Hints for Line Following:
    elif currentState == "line_follower" :
    
        if (gsr[0] < GROUND_SENSOR_THRESHOLD and gsr[1] < GROUND_SENSOR_THRESHOLD and gsr[2] < GROUND_SENSOR_THRESHOLD):
                if atStartLine == False:
                
                    startTiming = robot.getTime()
                    atStartLine = True
                elif onStartLine == False and (robot.getTime() - startTiming) > 0.1:
               
                    pose_x = 0
                    pose_y = 0
                    pose_theta = 0
                    print("Odometry reset")
                    onStartLine = True
        else:
                atStartLine = False
                onStartLine = False
        
        # start off with all sensors detecting line 
        if gsr[0] < GROUND_SENSOR_THRESHOLD and gsr[1] < GROUND_SENSOR_THRESHOLD and gsr[2] < GROUND_SENSOR_THRESHOLD:
            #print("striaght")
            vL = MAX_SPEED
            vR = MAX_SPEED
        # left and center detect black 
        elif gsr[0] < GROUND_SENSOR_THRESHOLD and gsr[1] < GROUND_SENSOR_THRESHOLD:
            #print("left and center")
            vL = 0.25 * MAX_SPEED
            vR = 0.5 * MAX_SPEED        
        # right and center detect black 
        elif gsr[2] < GROUND_SENSOR_THRESHOLD and gsr[1] < GROUND_SENSOR_THRESHOLD:
            #print("right and center")
            vL = 0.5 * MAX_SPEED
            vR = 0.25 * MAX_SPEED
        #center 
        elif gsr[1] < GROUND_SENSOR_THRESHOLD:
            #print("center")
            vL = 0.5 * MAX_SPEED
            vR = 0.5 * MAX_SPEED
        #left sensor 
        elif gsr[0] < GROUND_SENSOR_THRESHOLD:
            #print("left")
            #turn left 
            vL = -0.1 * MAX_SPEED
            vR = 0.1 * MAX_SPEED
        #right sensor 
        elif gsr[2] < GROUND_SENSOR_THRESHOLD: 
            #turn right 
            #print("right")
            vL = 0.1 * MAX_SPEED
            vR = -0.1 * MAX_SPEED
        #sensors detect nothing, so search for line 
        else :
            vL = -0.1 * MAX_SPEED
            vR = 0.1 * MAX_SPEED
        pose_x,pose_y,pose_theta = update_odometry(pose_x,pose_y,pose_theta, vL, vR)
        
    #
    # 1) Setting vL=MAX_SPEED and vR=-MAX_SPEED lets the robot turn
    # right on the spot. vL=MAX_SPEED and vR=0.5*MAX_SPEED lets the
    # robot drive a right curve.
    #
    # 2) If your robot "overshoots", turn slower.
    #
    # 3) Only set the wheel speeds once so that you can use the speed
    # that you calculated in your odometry calculation.
    #
    # 4) Disable all console output to simulate the robot superfast
    # and test the robustness of your approach.
    #
    # Hints for update_odometry:
    #
    # 1) Divide vL/vR by MAX_SPEED to normalize, then multiply with
    # the robot's maximum speed in meters per second.
    #
    # 2) SIM_TIMESTEP tells you the elapsed time per step. You need
    # to divide by 1000.0 to convert it to seconds
    #
    # 3) Do simple sanity checks. In the beginning, only one value
    # changes. Once you do a right turn, this value should be constant.
    #
    # 4) Focus on getting things generally right first, then worry
    # about calculating odometry in the world coordinate system of the
    # Webots simulator first (x points down, y points right)

    # Part 3
    # TODO: Implement Loop Closure also under state "line_follower" to reset pose when robot passes over the Start Line.
    # Hints:
    #
    # 1) Set a flag whenever you encounter the line
    #
    # 2) Use the pose when you encounter the line last
    # for best results

    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
