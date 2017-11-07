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


numberOfParticles = 100
initial_position = 100
final_position = 700
sleep_time = 0.25
step = 150

line1 = (initial_position, initial_position, initial_position, final_position) # (x0, y0, x1, y1)
line2 = (initial_position, initial_position, final_position, initial_position)
line3 = (final_position, initial_position, final_position, final_position)
line4 = (initial_position, final_position, final_position, final_position)

print "drawLine:" + str(line1)
print "drawLine:" + str(line2)
print "drawLine:" + str(line3)
print "drawLine:" + str(line4)

# Array of initial_position Particles of the form [x, y, th].
particles = [[0, 0, 0] for i in range(numberOfParticles)]

# Array of Weights corresponding to particles.
weights = [1 / numberOfParticles] * numberOfParticles

# mean and sigma parameters used by the update functions.
mu = 0.0
sigma = 4.0
sigma_a = 0.0075

# Updates uncertainty for Forward movement of 10 cm.
def UpdateParticlesAfterForward10(particles):
    D = 150
    for i in range(0, 100):
        particles[i][0] = particles[i][0] + (D + random.gauss(mu, sigma)) * math.cos(particles[i][2])
        particles[i][1] = particles[i][1] + (D + random.gauss(mu, sigma)) * math.sin(particles[i][2])
        particles[i][2] = particles[i][2] + random.gauss(mu, sigma_a)

    return particles

# Updates uncertainty for Left 90 movement.
def UpdateParticlesAfterLeft90(particles):
    for i in range(0, 100):
        particles[i][0] = particles[i][0]
        particles[i][1] = particles[i][1]
        particles[i][2] = particles[i][2] + math.pi/2 + random.gauss(mu, sigma_a)
    return particles

def Left90deg():
    print("Turning 90 left")
    angle = 4.235
    interface.increaseMotorAngleReferences(motors, [angle, -angle])
    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.1)

def Forward10():
    print("Forward 10")
    distance = 3.15
    interface.increaseMotorAngleReferences(motors, [-distance, -distance])
    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.1)


particles = [[initial_position, initial_position, 0] for i in range(numberOfParticles)]
particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
print "drawParticles:" + str(particles_new)
time.sleep(sleep_time)
for k in range(4):
    for c in range(step, final_position, step):
        Forward10()
        particles = UpdateParticlesAfterForward10(particles)
        print c, particles
        particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
        print "drawParticles:" + str(particles_new)
        time.sleep(sleep_time)
    Left90deg()
    particles = UpdateParticlesAfterLeft90(particles)
    particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles_new)
    time.sleep(sleep_time)

interface.terminate()
