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

# Array of Weights corresponding to particles.
weights = [0.01] * numberOfParticles

# mean and sigma parameters used by the update functions.
mu = 0.0
sigma = 5.0

x = 0
y = 0
th = 0

def meanValue(particles, weights):
    mean = [0, 0, 0]
    for i in range(0, 100):
        #mean += weights[i]*particles[i]
        mean[0] += weights[i]*particles[i][0]
        mean[1] += weights[i]*particles[i][1]
        mean[2] += weights[i]*particles[i][2]
    return mean

def translateActualRadians(angle):
    return 2.69608473048 * angle
    
def navigateToWaypoint(wx, wy):
    dx = wx - x
    dy = wy - y
    a = math.atan2(dy,dx)
    b = a - th
    dist = math.sqrt(math.pow(dx,2) + math.pow(dy,2))
    print b
    print a
    #if b > math.pi:
    #    b = b - 2*(math.pi)
    #    interface.increaseMotorAngleReferences(motors, [b, -b])
    #    while not interface.motorAngleReferencesReached(motors):
    #        time.sleep(0.1)
    
    #elif b <= -(math.pi):
    #    b = b + 2*(math.pi)
    #    interface.increaseMotorAngleReferences(motors, [-b, b])
    #    while not interface.motorAngleReferencesReached(motors):
    #        time.sleep(0.1)
    # horizontal right turn        
    if b == 0:
        Right90deg()
        Forward(dist)
    
    # forward
    elif b == (math.pi)/2:
        Forward(dist)
        
    # horizontal left turn
    elif b == math.pi:
        Left90deg()
        Forward(dist)

    # diagonal top right
    elif b > 0 and b < (math.pi)/2:
        TurndegDR(translateActualRadians(b))
        #Forward(dist)
        
    # diagonal top left
    elif b > (math.pi)/2 and b < math.pi:
        TurndegDL(translateActualRadians(b) - (math.pi)/2)
        Forward(dist)
        
    # diagonal bottom left
    elif b < -(math.pi)/2 and b > -(math.pi):
        TurndegDL(translateActualRadians(b + 2*(math.pi) - (math.pi)/2))
        Forward(dist)
        
    # diagonal bottom right     
    elif b < 0 and b > -(math.pi)/2:
        TurndegDL(translateActualRadians(b - (math.pi)/2))
        Forward(dist)
        

# Updates uncertainty for Forward movement of 10 cm.
def UpdateParticlesAfterForward10(particles):
    D = 3.0  # need to calculate accuretaly how many radians are 10cm.

    for i in range(0, len(particles)):
        particles[i][0] = particles[i][0] + (D + random.gauss(mu, sigma)) * math.cos(particles[i][2])
        particles[i][1] = particles[i][1] + (D + random.gauss(mu, sigma)) * math.sin(particles[i][2])
        particles[i][2] = particles[i][2] + random.gauss(mu, sigma)
    particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
    return particles_new

# Updates uncertainty for Left 90 movement.
def UpdateParticlesAfterLeft90(particles):
    for i in range(0, len(particles)):
        particles[i][0] = particles[i][0]
        particles[i][1] = particles[i][1]
        particles[i][2] = particles[i][2] + 90.0 + random.gauss(mu, sigma)
    particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
    return particles_new

def Left90deg():
    print("Turning 90 left")
    angle = 4.235
    interface.increaseMotorAngleReferences(motors, [angle, -angle])
    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.1)

def Right90deg():
    print("Turning 90 right")
    angle = 4.235
    interface.increaseMotorAngleReferences(motors, [-angle, angle])
    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.1)

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
    
def Forward10():
    print("Forward 10")
    distance = 3.15
    interface.increaseMotorAngleReferences(motors, [-distance, -distance])
    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.1)
        
def Forward(d):
    print("Forward " + str(d))
    interface.increaseMotorAngleReferences(motors, [-d, -d])
    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.1)

#k = 0
#while (k < 4):
#    for c in range(150, final_position, step):
#        if k == 0:
#            particles[0] = [initial_position + c, initial_position, 0]
#        elif k == 1:
#            particles[0] = [final_position, initial_position + c, 90]
#        elif k == 2:
#             particles[0] = [final_position - c, final_position, 180]
#        elif k == 3:
#             particles[0] = [initial_position, final_position - c, 270]
#        Forward10()
#        particles_new = UpdateParticlesAfterForward10(particles)
#        print "drawParticles:" + str(particles_new)
#       time.sleep(sleep_time)
#    Left90deg()
#    part_new = UpdateParticlesAfterLeft90(particles)
#    print "drawParticles:" + str(part_new)
#    time.sleep(sleep_time)
#    k += 1

#print meanValue(particles, weights)

# right top
navigateToWaypoint(5, -5)

# left top
#navigateToWaypoint(-3,4)

# right down
#navigateToWaypoint(3,-4)

# left down
#navigateToWaypoint(-3,-4)

interface.terminate()
