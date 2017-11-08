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

# mean and sigma parameters used by the update functions.
mu = 0.0
sigma = 5.0

def ResetParticles():
    particles = [[0,0,0]] * NUMBER_OF_PARTICLES

def ShowParticles(particles):
    for p in particles:
        print p
    print ''

# Updates uncertainty for Forward movement of 10 cm.
def UpdateParticlesAfterForward10():
    D = 3.0
    e = random.gauss(mu, sigma)
    f = random.gauss(mu, sigma_a)
    for particle in particles:
        particle[0] = particle[0] + (D + e) * math.cos(particle[2])
        particle[1] = particle[1] + (D + e) * math.sin(particle[2])
        particle[2] = particle[2] + f

    return particles

# Updates uncertainty for Left 90 movement.
def UpdateParticlesAfterLeft90():
    g = random.gauss(mu, sigma_a)
    for particle in particles:
        particle[2] = particle[2] + math.pi/2 + g
    return particles

def Square():
    # Setting all particles to starting position [0,0,0].
    ResetParticles()

    # First forward movement
    Forward10()
    Forward10()
    Forward10()
    Forward10()

    # First Left
    Left90deg()

    # Second forward movement
    Forward10()
    Forward10()
    Forward10()
    Forward10()

    # Second left
    Left90deg()

    # Third forward movement
    Forward10()
    Forward10()
    Forward10()
    Forward10()

    # Last left
    Left90deg()

    # Last forward
    Forward10()
    Forward10()
    Forward10()
    Forward10()

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


########## PROGRAM EXECUTION ##########

#logfile = raw_input("Specify logfile: ")
#interface.startLogging("/home/pi/BrickPi/Logfiles/" + logfile)



#interface.stopLogging()
interface.terminate()
