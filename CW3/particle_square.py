import brickpi  
import time
import random

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

motorParamsRight = motorParams
motorParamsLeft = motorParams

motorParamsLeft.pidParameters.k_p = 250
motorParamsLeft.pidParameters.k_i = 100
motorParamsLeft.pidParameters.k_d = 0

motorParamsRight.pidParameters.k_p = 250
motorParamsRight.pidParameters.k_i = 100
motorParamsRight.pidParameters.k_d = 0

interface.setMotorAngleControllerParameters(motors[0],motorParamsLeft)
interface.setMotorAngleControllerParameters(motors[1],motorParamsRight)

# Number of Particles predefined as 100.
NUMBER_OF_PARTICLES = 100

# Array of 100 Particles of the form [x, y, th].
particles = [[0,0,0]] * NUMBER_OF_PARTICLES

# Array of Weights corresponding to particles.
weights = [1 / NUMBER_OF_PARTICLES] * NUMBER_OF_PARTICLES

# Updates uncertainty for Forward movement of 10 cm
def UpdateParticlesAfterForward10(particles)
    x = particle[0]
    y = particle[1]
    th = particle[2]
    D = 3 # need to calculate this correctly in radians.
    mu = 3  # need to find this as mentioned in the spec.
    sigma = 5   # and this as well.
    
    for particle in particles:
            x = x + (D + randmo.gauss(mu, sigma)) * 
            y = y + (D + randmo.gauss(mu, sigma)) * 
            th = th + randmo.gauss(mu, sigma)

def Left90deg():
    print("Turning 90 left")
    angle = 4.85
    interface.increaseMotorAngleReferences(motors, [angle, -angle])
    while not interface.motorAngleReferencesReached(motors): 
	    time.sleep(0.1) 
    
def Forward10():
    print("Forward 10")
    distance = 3.0
    interface.increaseMotorAngleReferences(motors, [-distance, -distance])
    while not interface.motorAngleReferencesReached(motors):  
	    time.sleep(0.1)    

logfile = raw_input("Specify logfile: ")
interface.startLogging("/home/pi/BrickPi/Logfiles/" + logfile)

# testing the particles
#print particles[0]
#print particles[0][0]
#print weights[0]

# First forward movement
Forward10()
#Forward10()
#Forward10()
#Forward10()

# First Left
#Left90deg()

# Second forward movement
#Forward10()
#Forward10()
#Forward10()
#Forward10()

# Second left
#Left90deg()

# Third forward movement
#Forward10()
#Forward10()
#Forward10()
#Forward10()

# Last left
#Left90deg()

# Last forward
#Forward10()
#Forward10()
#Forward10()
#Forward10()

interface.stopLogging()
interface.terminate()
