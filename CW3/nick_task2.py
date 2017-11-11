import time
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


numberOfParticles = 100
initial_position = 100
final_position = 700
sleep_time = 0.5
step = 150

line1 = (initial_position, initial_position, initial_position, final_position) # (x0, y0, x1, y1)
line2 = (initial_position, initial_position, final_position, initial_position)
line3 = (final_position, initial_position, final_position, final_position)
line4 = (initial_position, final_position, final_position, final_position)

# Array of initial_position Particles of the form [x, y, th].
particles = [[0, 0, 0] for i in range(numberOfParticles)]

#Initial pos
init_pos = [0, 0, math.pi/2]

# Array of Weights corresponding to particles.
weights = [0.01] * numberOfParticles

# mean and sigma parameters used by the update functions.
mu = 0.0
sigma = 5.0
sigma_a = 0.075

x = 0
y = 0
th = 0


def navigateToWaypoint(wx, wy):
    dx = wx - init_pos[0]
    dy = wy - init_pos[1]
    th = init_pos[2]
    
    
    print "dx, dy: " + str(dx) +  " " + str(dy)
    
    #calculates the angle of a new vector
    a = (math.atan2(dy,dx) + 2*math.pi) % (2*math.pi)
    #calculates angle to turn to get to the new position
    b = (a - th + 2*math.pi) % (2*math.pi)
    #arbitary unit distance proportional to coordinate system
    dist = math.sqrt(math.pow(dx,2) + math.pow(dy,2))
    #determines if its a right or left rotation
    if b > math.pi:
         b = b - 2*math.pi
    #translates the angle to turn to the wheel angle
    angle = (b*4.135*2)/math.pi
    print "Global x, y, th: " + str(x) + " " + str(y) + " " +  str(th)
    #moves the robot
    TurndegDL(angle)
    Forward(dist*2)
    return [wx, wy, a]

def TurndegDR(angle):
    print("Turning right " + str(angle))
    interface.increaseMotorAngleReferences(motors, [-angle, angle])
    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.1)
        
def TurndegDL(angle):
    print("Turning " + str(angle))
    interface.increaseMotorAngleReferences(motors, [angle, -angle])
    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.1)
    
        
def Forward(d):
    print("Forward " + str(d))
    interface.increaseMotorAngleReferences(motors, [-d, -d])
    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.1)

        
# left top
#init_pos =navigateToWaypoint(0,1)

# right down
#init_pos =navigateToWaypoint(1,1)

# left down
#init_pos =navigateToWaypoint(1,-1)

#init_pos = navigateToWaypoint(0,-1)
#init_pos = navigateToWaypoint(0,0)
init_pos =navigateToWaypoint(3,4)
init_pos =navigateToWaypoint(6,-8)
init_pos =navigateToWaypoint(-6,-13)
init_pos =navigateToWaypoint(0,0)





interface.terminate()

