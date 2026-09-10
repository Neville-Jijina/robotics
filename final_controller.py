"""epuck_lab1_code controller."""

from controller import Robot, DistanceSensor, Motor

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

ls = []
lsNames = ['ls0', 'ls1', 'ls2', 'ls3', 'ls4', 'ls5', 'ls6', 'ls7']

for i in range(len(lsNames)):
    ls.append(robot.getDevice(lsNames[i]))
    ls[i].enable(TIME_STEP)
   

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

vL = 0
vR = 0

#robot starts off following left wall 
currentState = "FOLLOW_LEFT"

#counting for how long robot is turning for 
turnCounter = 0

# keeps track of whether robot is currently going around the box
leftWallWasSeen = False

#Light sensor reading 
light_seen = 75

def light_found(lsValues):
    light_sensors = [0,1,2,5,6,7]
    return all(lsValues[i] < light_seen for i in light_sensors)



# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
   
    lsValues = []
    for i in range(8):
        lsValues.append(ls[i].getValue())
    
    
    light_detected = light_found(lsValues)
        
    # state machine 
    if currentState == "FOLLOW_LEFT":
    
        if light_detected:
            currentState = "TURN_180"
            print("Light Source #1 Found!")
            print("Starting Right Wall Follow")
            turnCounter = 0
            vL = 0.0
            vR= 0.0
        # remember when there is wall on the left
        if psValues[5] > 100:
            leftWallWasSeen = True
            
        if psValues[0] > 120 or psValues[7] > 120: 
            #if there is an obstacle in the front (the end of the wall)
            currentState = "RIGHT_TURN"
            turnCounter = 0 #reset to 0
            
        elif psValues[5] > 200:
            #this range is too close to left wall 
            #move robot to the right 
            vL = 0.5 * MAX_SPEED
            vR = -0.2 * MAX_SPEED
        elif psValues[5] > 230:
            #this range is too close to left wall 
            #move robot to the right 
            vL = 0.5 * MAX_SPEED
            vR = -0.5 * MAX_SPEED
            
        elif psValues[5] < 150 and psValues[5] > 120:
            vL = 0.5 * MAX_SPEED
            vR = 0.5 * MAX_SPEED
            
        elif psValues[5] < 120 and psValues[5] > 80:
            #turn left
            vL = 0.4 * MAX_SPEED
            vR = 0.5 * MAX_SPEED
            
        elif psValues[5] < 80 and psValues[5] > 70:
            #this range is too far from left wall 
            #move robot to the left
            # currentState = "LEFT_TURN"
            vL = 0.4 * MAX_SPEED
            vR = 0.5 * MAX_SPEED
            
        elif (psValues[6] < 70 and psValues[5] < 70):
            if leftWallWasSeen:
               vL = -0.5 * MAX_SPEED
               vR = 0.5 * MAX_SPEED
            
            
        elif psValues[5] < 70:
            #go straight till you find something
            if leftWallWasSeen == False:
                vL = 0.5 * MAX_SPEED
                vR = 0.5 * MAX_SPEED

        else: 
            #go straight
            vL = 0.5 * MAX_SPEED
            vR = 0.5 * MAX_SPEED
            
    
    elif currentState == "RIGHT_TURN" : 
        # turn right!
        vL = 0.5 * MAX_SPEED
        vR = -0.5 * MAX_SPEED
        
        turnCounter += 1
        
        if turnCounter >= 12 :
            currentState = "FOLLOW_LEFT"
            turnCounter = 0
            
    elif currentState == "LEFT_TURN":
        vL = -0.5 * MAX_SPEED
        vR = 0.5 * MAX_SPEED
        
        turnCounter += 1
        if turnCounter >= 12 :
            currentState = "FOLLOW_LEFT"
            turnCounter = 0
            
    elif currentState == "RIGHT_TURN_RIGHT" : 
        # turn right!
        print("Right Turn")
        vL = 0.5 * MAX_SPEED
        vR = -0.5 * MAX_SPEED
        
        turnCounter += 1
        
        if turnCounter >= 12 :
            currentState = "FOLLOW_RIGHT"
            turnCounter = 0
            
    elif currentState == "LEFT_TURN_RIGHT":
        print("Turning LEFT")
        vL = -0.5 * MAX_SPEED
        vR = 0.5 * MAX_SPEED
        
        turnCounter += 1
        if turnCounter >= 12 :
            currentState = "FOLLOW_RIGHT"
            turnCounter = 0
        
    elif currentState == "TURN_180":
        print("Turning 180...")
        vL = 0.5 * MAX_SPEED
        vR = -0.5 * MAX_SPEED

        turnCounter += 1

        if turnCounter >= 24:
            currentState = "FOLLOW_RIGHT"
            turnCounter = 0
    
    elif currentState == "FOLLOW_RIGHT":
    
        if light_detected:
            currentState = "STOP"
            print("Light Source #2 Found!")
            print("Found second light source, Job Complete!")
            turnCounter = 0
            vL = 0.0
            vR= 0.0
        # remember when there is wall on the left
        if psValues[2] > 100:
            rightWallWasSeen = True
            
        if psValues[0] > 120 or psValues[7] > 120: 
            #if there is an obstacle in the front (the end of the wall)
            currentState = "LEFT_TURN_RIGHT"
            turnCounter = 0 #reset to 0
            
        elif psValues[2] > 200:
            #this range is too close to left wall 
            #move robot to the right 
            vL = -0.2 * MAX_SPEED
            vR = 0.5 * MAX_SPEED
        elif psValues[2] > 230:
            #this range is too close to left wall 
            #move robot to the right 
            vL = -0.5 * MAX_SPEED
            vR = 0.5 * MAX_SPEED
            
        elif psValues[2] < 150 and psValues[2] > 120:
            vL = 0.5 * MAX_SPEED
            vR = 0.5 * MAX_SPEED
            
        elif psValues[2] < 120 and psValues[2] > 80:
            #turn right
            vL = 0.5 * MAX_SPEED
            vR = 0.4 * MAX_SPEED
            
        elif psValues[2] < 80 and psValues[2] > 70:
            #this range is too far from left wall 
            #move robot to the left
            # currentState = "LEFT_TURN"
            vL = 0.5 * MAX_SPEED
            vR = 0.4 * MAX_SPEED
            
        elif (psValues[1] < 70 and psValues[2] < 70):
            if rightWallWasSeen:
               print("MAX left turn")
               vL = 0.5 * MAX_SPEED
               vR = -0.5 * MAX_SPEED
            
            
        elif psValues[2] < 70:
            #go straight till you find something
            if rightWallWasSeen == False:
                vL = 0.5 * MAX_SPEED
                vR = 0.5 * MAX_SPEED

        else: 
            #go straight
            vL = 0.5 * MAX_SPEED
            vR = 0.5 * MAX_SPEED
    
    
    elif currentState == "STOP" : 
        vL = 0
        vR = 0
        

    # write actuators inputs
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
