"""lab5 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as pyplot

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 2.75 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)

wall_distance_thresh = 0.5
wall_detect_thresh = 1.0

SPEED_SCALE = 0.5
TARGET = 0.35
KP = 0.6
in_corner = False
corner_ticks = 0
corner_start_theta = 0.0   


#constants for spin
spin_every = 700 #loop steps between scans since 250*32 ms = 8 sec ish 
still_steps = 15 # steps that robot is still for before spinning 
spin_steps = 225 
spin_speed = 0.3*MAX_SPEED
counter = 0

##### vvv [Begin] Do Not Modify vvv #####

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# The Tiago robot has a couple more sensors than the e-Puck
# Some of them are mentioned below. We will use its LiDAR for Lab 5

range = robot.getDevice('range-finder')
range.enable(timestep)
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# The display is used to display the map. We are using 360x360 pixels to
# map the 12x12m2 apartment
display = robot.getDevice("display")

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

# map = None
##### ^^^ [End] Do Not Modify ^^^ #####

##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
 # Part 1.1: manual mode
# mode = 'manual'
mode = 'autonomous'

######################
#
# Map Initialization
#
######################

# Part 1.2: Map Initialization
#Find Wall Follow Wall

state = "FIND_WALL" 
wall_direction = "FRONT"

# Initialize your map data structure here as a 2D floating point array
map = np.zeros(shape=[360,360])
waypoints = []

while robot.step(timestep) != -1 and mode != 'planner':

    ###################
    #
    # Mapping
    #
    ###################

    ################ v [Begin] Do not modify v ##################
    # Ground truth pose
    pose_x = gps.getValues()[0]
    pose_y = gps.getValues()[1]
    
    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        t = pose_theta + np.pi/2.
        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(t)*rx - math.sin(t)*ry + pose_x
        wy =  math.sin(t)*rx + math.cos(t)*ry + pose_y

        ################ ^ [End] Do not modify ^ ##################

        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))
        if wx >= 12:
            wx = 11.999
        if wy >= 12:
            wy = 11.999
        if rho < LIDAR_SENSOR_MAX_RANGE:
            # Part 1.3: visualize map gray values.
            #converting the world coords to map pixel coords
            px = 360 - abs(int(wx *30))
            py = abs(int(wy *30))
            
            #make sure that the coords are within the map 
            if 0 <= px < 360 and 0 <= py < 360: 
                #increase the map value each time an obstcle is detected
                #the max value is 1
                map[py,px] = min(map[py,px] + 0.005, 1.0) 
                
                #convert the map value to grayscale 
                g = map[py,px]
                gray = int(g * 255)# converting to integer first 
                color =(gray*256**2 + gray*256 + gray)
                
                #draw grayscale pixel 
                display.setColor(color)
                display.drawPixel(px,py) 
 
            # You will eventually REPLACE the following lines with a more robust version of the map
            # with a grayscale drawing containing more levels than just 0 and 1.
            # display.setColor(int(0X0000FF))
            # display.drawPixel(360-abs(int(wx*30)),abs(int(wy*30)))

    # Draw the robot's current pose on the 360x360 display
    display.setColor(int(0xFF0000))
    display.drawPixel(360-abs(int(pose_x*30)), abs(int(pose_y*30)))
    

    ###################
    #
    # Controller
    #
    ###################
    if mode == 'manual':
        #Perform teleoperation and obtain a map of the entire environment and store the map as an npy file
        key=keyboard.getKey()
        if (key==Keyboard.UP):
            print("up")
            vL = 0.5 * MAX_SPEED
            vR = 0.5 * MAX_SPEED
        elif (key==Keyboard.DOWN):
            print("down")
            vL = -0.5 * MAX_SPEED
            vR = -0.5 * MAX_SPEED
        elif (key==Keyboard.LEFT):
            print("left")
            vL = -0.5 * MAX_SPEED
            vR = 0.5 * MAX_SPEED
        elif (key==Keyboard.RIGHT):
            print("right")
            vL = 0.5 * MAX_SPEED
            vR = -0.5 * MAX_SPEED
        elif (key== ord('S')):
            #save map as NumPy file 
            #also implements 
            np.save("map.npy", (map > 0.5).astype(float))
            print("Map saved")
            vL = 0 
            vR = 0
        else:
            vL = 0 * MAX_SPEED
            vR = 0 * MAX_SPEED
            
            

    elif mode == 'autonomous':
        print("state:", state, " counter:", counter, " in_corner:", in_corner)

        if state == "FIND_WALL":
            print("Finding Wall")
            front = []
            front_right = []  
            left = []
            right = []

            counter += 1
            if counter >= spin_every:
                counter = 0
                state = "SPIN"

            for i, rho in enumerate(lidar_sensor_readings):
                alpha = lidar_offsets[i]

                if abs(alpha) <= math.radians(15):
                    front.append(rho)

                elif abs(alpha - math.radians(40)) <= math.radians(20):
                    front_right.append(rho)

                elif abs(alpha + math.pi/2) <= math.radians(25):
                    left.append(rho)

                elif abs(alpha - math.pi/2) <= math.radians(25):
                    right.append(rho)

            # If lidars go past max it can cause issues so replace it with the max value instead
            front_distance = min(front) if front else LIDAR_SENSOR_MAX_RANGE
            front_right_distance = min(front_right) if front_right else LIDAR_SENSOR_MAX_RANGE  
            left_distance = min(left) if left else LIDAR_SENSOR_MAX_RANGE
            right_distance = min(right) if right else LIDAR_SENSOR_MAX_RANGE
            
            closest_distance = min(front_distance,left_distance,right_distance)

            narrow = (left_distance + right_distance) < 1.2

            if front_distance < (0.6 if narrow else 0.9):
                print("blocked ahead, turning left")
                vL = -0.2 * MAX_SPEED
                vR = 0.5 * MAX_SPEED

            elif right_distance < (0.15 if narrow else 0.2):
                print("CRITICAL: right wall too close, spinning away")
                vL = -0.3 * MAX_SPEED
                vR = 0.5 * MAX_SPEED

            elif narrow and left_distance < 0.15:
                print("CRITICAL: left wall too close, steering right")
                vL = 0.5 * MAX_SPEED
                vR = -0.3 * MAX_SPEED

            elif right_distance < 0.8 and front_right_distance < 1.0 * right_distance:
                print("protrusion ahead on right, steering left")
                vL = 0.1 * MAX_SPEED
                vR = 0.4 * MAX_SPEED

            elif narrow:
                print("narrow space, centering")
                error = right_distance - left_distance
                steer = max(-0.15, min(0.15, 0.5 * error))
                vL = (0.3 + steer) * MAX_SPEED
                vR = (0.3 - steer) * MAX_SPEED

            elif right_distance > 1.2:
                print("outside corner, arcing right")
                vL = 0.5 * MAX_SPEED
                vR = -0.5 * MAX_SPEED

            elif front_distance > 0.8 and right_distance > 0.8 and left_distance > 0.9:
                print("lost wall, curving right to find it")
                vL = 0.4 * MAX_SPEED
                vR = 0.1 * MAX_SPEED

            else:
                error = right_distance - TARGET        
                if abs(error) < 0.04:
                    error = 0.0                        
                steer = max(-0.15, min(0.15, KP * error))
                print("following wall, error:", round(error, 2))
                vL = (0.3 + steer) * MAX_SPEED        
                vR = (0.3 - steer) * MAX_SPEED

            vL *= SPEED_SCALE
            vR *= SPEED_SCALE

        elif state == "FOLLOW_WALL":
            front = []
            front_right = []
            left = []
            right = []

            for i, rho in enumerate(lidar_sensor_readings):
                alpha = lidar_offsets[i]

                if abs(alpha) <= math.radians(15):
                    front.append(rho)

                elif abs(alpha - math.radians(40)) <= math.radians(20):
                    front_right.append(rho)

                elif abs(alpha + math.pi/2) <= math.radians(25):
                    left.append(rho)

                elif abs(alpha - math.pi/2) <= math.radians(25):
                    right.append(rho)

            front_distance = min(front) if front else LIDAR_SENSOR_MAX_RANGE
            front_right_distance = min(front_right) if front_right else LIDAR_SENSOR_MAX_RANGE
            left_distance = min(left) if left else LIDAR_SENSOR_MAX_RANGE
            right_distance = min(right) if right else LIDAR_SENSOR_MAX_RANGE

            print("Front Distance: ", front_distance)
            print("Front-Right Distance: ", front_right_distance)
            print("Right Distance: ", right_distance)
            print("Left Distance: ", left_distance)

            if not in_corner and right_distance > 1.2:
                in_corner = True
                corner_ticks = 0
                corner_start_theta = pose_theta
                print(">>> corner started")
            elif in_corner:
                corner_ticks += 1
                d = pose_theta - corner_start_theta
                turned = abs(math.atan2(math.sin(d), math.cos(d)))
                print("corner: turned", round(math.degrees(turned)), "deg")
                if turned > math.radians(80) or corner_ticks > 600:
                    in_corner = False
                    print("<<< corner finished")

            if in_corner:
                if front_distance < 0.45:
                    vL = 0.2 * MAX_SPEED
                    vR = -0.2 * MAX_SPEED
                elif right_distance < 0.25:
                    print("too close to wall")
                    vL = 0.25 * MAX_SPEED
                    vR = 0.25 * MAX_SPEED
                else:
                    vL = 0.3 * MAX_SPEED
                    vR = 0.1 * MAX_SPEED

            elif front_distance < 0.6:
                print("blocked ahead, turning left")
                vL = -0.2 * MAX_SPEED
                vR = 0.5 * MAX_SPEED

            elif right_distance < 0.2:
                print("right wall too close")
                vL = -0.3 * MAX_SPEED
                vR = 0.5 * MAX_SPEED

            elif left_distance < 0.2:
                print("left wall too close")
                vL = 0.5 * MAX_SPEED
                vR = -0.3 * MAX_SPEED

            
            elif right_distance < 0.8 and front_right_distance < 0.8 * right_distance:
                vL = 0.1 * MAX_SPEED
                vR = 0.4 * MAX_SPEED

            elif front_distance > 0.8 and right_distance > 0.8:
                vL = 0.4 * MAX_SPEED
                vR = 0.1 * MAX_SPEED

            else:
                error = right_distance - TARGET
                if abs(error) < 0.04:
                    error = 0.0
                steer = max(-0.15, min(0.15, KP * error))
                vL = (0.3 + steer) * MAX_SPEED
                vR = (0.3 - steer) * MAX_SPEED

            vL *= SPEED_SCALE
            vR *= SPEED_SCALE

            counter += 1
            if counter >= spin_every and not in_corner:
                counter = 0
                state = "SPIN"
            
        elif state == "SPIN":
            print("spinning")
            counter += 1
            
            if counter <= still_steps:
            #stand still for a lil first
                vL = 0
                vR = 0
            elif counter <= still_steps + spin_steps:
                #rotate in place
                vL = -spin_speed
                vR = spin_speed
            else:
                #go back to wall following 
                vL= 0 
                vR=0
                counter = 0
                state = "FOLLOW_WALL"
            
            
            
        #Autonomously explore the entire environment, generate a map, and store the map as an npy file
        # Odometry code. Don't change vL or vR speeds after this line.
        # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    # print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta))
    
    # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
    
while robot.step(timestep) != -1:
    # there is a bug where webots have to be restarted if the controller exits on Windows
    # this is to keep the controller running
    pass