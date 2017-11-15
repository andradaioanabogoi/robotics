mport time
import sys
import random
import math
import brickpi 

interface=brickpi.Interface()
interface.initialize()

motors = [0,3]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 250
motorParams.pidParameters.k_i = 225
motorParams.pidParameters.k_d = 0

motorParamsRight = motorParams
motorParamsLeft = motorParams


interface.setMotorAngleControllerParameters(motors[0],motorParamsLeft)
interface.setMotorAngleControllerParameters(motors[1],motorParamsRight)

# mean and sigma parameters used by the update functions.
mu = 0.0
sigma = 5.0
sigma_a = 0.075

counter = 1

#radius for 20cm 
r = 3.35

def ForwardCm(d):
    print("Moving " + str(d))
    x = d/r
    interface.increaseMotorAngleReferences(motors, [-x, -x])
    while not interface.motorAngleReferencesReached(motors):
            time.sleep(0.1)

def TurndegDL(angle):
    print("Turning " + str(angle))
    interface.increaseMotorAngleReferences(motors, [angle, -angle])
    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.1)


#Initial pos
init_pos = [84, 30, 0]
cur_pos = init_pos
        
def navigateToWaypoint(wx, wy):
    #========commented out code for the new movement=========
    #init_pos
    #try to navigate to point from init_pos with max of 20cm at a time    
    #while not reached
    while cur_pos[0] != wx or cur_pos[1] != wy:
        print("Cur_pos " + str(cur_pos))        
        dx = wx - cur_pos[0]
        dy = wy - cur_pos[1]
        th = init_pos[2]
    
        #=======================rotates======================
        
        #calculates the angle of a new vector
        a = (math.atan2(dy,dx) + 2*math.pi) % (2*math.pi)
        b = (a - th + 2*math.pi) % (2*math.pi)
    
        #determines if its a right or left rotation
        b1 = b
        if b > math.pi:
             b1 = b - 2*math.pi
    
        #translates the angle to turn to the wheel angle
        #3.777 is very good for 90* left
        angle = (b1*3.777*2)/math.pi
    
        #rotates the robot
        TurndegDL(angle)
    
        #===============make a move==========================
        
        #calculate dist in cm to target point 
        dist = math.sqrt(math.pow(dx,2) + math.pow(dy,2))
        
        #move 20 or less
        if dist >= 20:
            dist = 20
            ForwardCm(20)
        else:
            ForwardCm(dist)

        #===============cur_pos update======================
        
        cur_pos[2] = a
        cur_pos[0] += dist
        #TODO: update cur_pos with mean of particles
        
        time.sleep(0.9)
        
    global counter    
    print("Point " + str(counter) + " reached")  
    counter += 1
    #return [wx, wy, cur_pos[2]]
#TurndegDL(3.777)
#time.sleep(1.0)
#TurndegDL(-3.9)
#ForwardCm(20)

#navigateToWaypoint(84,30)
navigateToWaypoint(180,30)
#navigateToWaypoint(180,54)
#init_pos =navigateToWaypoint(138,54)
#init_pos =navigateToWaypoint(138,168)
#init_pos =navigateToWaypoint(114,168)
#init_pos =navigateToWaypoint(114,84)
#init_pos =navigateToWaypoint(84,84)
#init_pos =navigateToWaypoint(84,30)


interface.terminate()

