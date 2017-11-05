import time
import sys
import random
import math

c = 0;
def getRandomX():
    return random.randint((c%10)*40, (c%10 + 1)*40)

def getRandomY():
    return random.randint((c%10)*40, (c%10 + 1)*40)

def getRandomTheta():
    return random.randint(0, 360)

numberOfParticles = 100

line1 = (10, 10, 10, 400) # (x0, y0, x1, y1)
line2 = (10, 10, 400, 10)
line3 = (400, 10, 400, 400)
line4 = (10, 400, 400, 400)

print "drawLine:" + str(line1)
print "drawLine:" + str(line2)
print "drawLine:" + str(line3)
print "drawLine:" + str(line4)

# Array of 100 Particles of the form [x, y, th].
particles = [[0,0,0]] * numberOfParticles

# Array of Weights corresponding to particles.
weights = [1 / numberOfParticles] * numberOfParticles

# mean and sigma parameters used by the update functions.
mu = 0.0
sigma = 5.0

D = 0

def updateX(i):
    return particles[i-1][0] + (D + random.gauss(mu, sigma)) * math.cos(particles[i-1][2])

def updateY(i):
    return particles[i-1][1] + (D + random.gauss(mu, sigma)) * math.sin(particles[i-1][2])

def updateTheta(i):
    return particles[i-1][2] + (D + random.gauss(mu, sigma))

def updateXTurn(i):
    return particles[i-1][0]

def updateYTurn(i):
    return particles[i-1][1]

def updateThetaTurn(i):
    return particles[i-1][2] + 90 + random.gauss(mu, sigma)

# while True:
    # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
    # particles = [(getRandomX(), getRandomY(), getRandomTheta()) for i in range(numberOfParticles)]
    # particles_new = [(updateXTurn(i), updateYTurn(i), updateThetaTurn(i)) for i in range(1, numberOfParticles)]

def updatePoints():
    D = 0
    while D < 400 :
        particles = [(updateX(i), updateY(i), updateTheta(i)) for i in range(1, numberOfParticles)]
        print "drawParticles:" + str(particles)

        D += 100;
        time.sleep(0.25)

        #particles = [(updateXTurn(i), updateYTurn(i), updateThetaTurn(i)) for i in range(1, numberOfParticles)]
        #print "drawParticles:" + str(particles)
        # time.sleep(0.25)

particles = [[0,0,0]] * numberOfParticles
updatePoints()

particles = [[400,0,90]] * numberOfParticles
updatePoints()

particles = [[400,400,180]] * numberOfParticles
updatePoints()

particles = [[400,400,270]] * numberOfParticles
updatePoints()
