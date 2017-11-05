import brickpi  
import time
import random
import math

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
motorParamsLeft.pidParameters.k_i = 225
motorParamsLeft.pidParameters.k_d = 0

motorParamsRight.pidParameters.k_p = 250
motorParamsRight.pidParameters.k_i = 225
motorParamsRight.pidParameters.k_d = 0

interface.setMotorAngleControllerParameters(motors[0],motorParamsLeft)
interface.setMotorAngleControllerParameters(motors[1],motorParamsRight)

# Number of Particles predefined as 100.
NUMBER_OF_PARTICLES = 100

# Array of 100 Particles of the form [x, y, th].
particles = [[0,0,0]] * NUMBER_OF_PARTICLES

# Array of Weights corresponding to particles.
weights = [1 / NUMBER_OF_PARTICLES] * NUMBER_OF_PARTICLES

# Updates uncertainty for Forward movement of 10 cm.
def UpdateParticlesAfterForward10(particles):
    D = 3 # need to calculate accuretaly how many radians are 10cm.
    mu = 0  # in the middle of spec page 2 says something about mean = 0.
    sigma = 5   # but need to find this standard deviation from real execution.
    
    for particle in particles:
        particle[0] = particle[0] + (D + randmo.gauss(mu, sigma)) * math.cos(particle[2])
        particle[1] = particle[1] + (D + randmo.gauss(mu, sigma)) * math.sin(particle[2])
        particle[2] = particle[2] + randmo.gauss(mu, sigma)

# Updates uncertainty for Left 90 movement.            
def UpdateParticlesAfterLeft90(particles):
    for particle in particles:
        particle[2] = particle[2] + randmo.gauss(mu, sigma) + randmo.gauss(mu, sigma)    
            
def Left90deg():
    print("Turning 90 left")
    angle = 4.55
    interface.increaseMotorAngleReferences(motors, [angle, -angle])
    while not interface.motorAngleReferencesReached(motors): 
	    time.sleep(0.1) 
    
def Forward10():
    print("Forward 10")
    distance = 3.0
    interface.increaseMotorAngleReferences(motors, [-distance, -distance])
    while not interface.motorAngleReferencesReached(motors):  
	    time.sleep(0.1)    

#logfile = raw_input("Specify logfile: ")
#interface.startLogging("/home/pi/BrickPi/Logfiles/" + logfile)

# testing the particles
#print particles[0]
#print particles[0][0]
#print weights[0]

# First forward movement
#Forward10()
#UpdateParticlesAfterForward10(particles)
#Forward10()
#UpdateParticlesAfterForward10(particles)
#Forward10()
#UpdateParticlesAfterForward10(particles)
#Forward10()

# First Left
Left90deg()
#UpdateParticlesAfterLeft90(particles)

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

#interface.stopLogging()
interface.terminate()
