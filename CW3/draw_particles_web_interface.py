import time
import sys
import random
import math

numberOfParticles = 100
initial_position = 100
final_position = 700
sleep_time = 0.5
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
sigma = 5.0

# Updates uncertainty for Forward movement of 10 cm.
def UpdateParticlesAfterForward10(particles):
    D = 3.0  # need to calculate accuretaly how many radians are 10cm.

    for i in range(1, len(particles)):
        particles[i][0] = particles[i-1][0] + (D + random.gauss(mu, sigma)) * math.cos(particles[i-1][2])
        particles[i][1] = particles[i-1][1] + (D + random.gauss(mu, sigma)) * math.sin(particles[i-1][2])
        particles[i][2] = particles[i-1][2] + random.gauss(mu, sigma)
    return particles

# Updates uncertainty for Left 90 movement.
def UpdateParticlesAfterLeft90(particles):
    for i in range(1, len(particles)):
        particles[i][0] = particles[i-1][0]
        particles[i][1] = particles[i-1][1]
        particles[i][2] = particles[i-1][2] + 90.0 + random.gauss(mu, sigma)
    return particles

k = 0
while (k < 4):
    for c in range(0, final_position, step):
        if k == 0:
            particles[0] = [initial_position + c, initial_position, 0]
        elif k == 1:
            particles[0] = [final_position, initial_position + c, 90]
        elif k == 2:
             particles[0] = [final_position - c, final_position, 180]
        elif k == 3:
             particles[0] = [initial_position, final_position - c, 270]
        particles = UpdateParticlesAfterForward10(particles)
        particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
        print "drawParticles:" + str(particles_new)
        time.sleep(sleep_time)

    particles = UpdateParticlesAfterLeft90(particles)
    particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles_new)
    time.sleep(sleep_time)
    k += 1


# for c in range(0, final_position, step):
#     particles[0] = [initial_position + c, initial_position, 0]
#     particles = UpdateParticlesAfterForward10(particles)
#     particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
#     print "drawParticles:" + str(particles_new)
#     time.sleep(sleep_time)
#
# particles = UpdateParticlesAfterLeft90(particles)
# particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
# print "drawParticles:" + str(particles_new)
# time.sleep(sleep_time)
#
# for c in range(0, final_position, step):
#     particles[0] = [final_position, initial_position + c, 90]
#     particles = UpdateParticlesAfterForward10(particles)
#     particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
#     print "drawParticles:" + str(particles_new)
#     time.sleep(sleep_time)
#
# particles = UpdateParticlesAfterLeft90(particles)
# particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
# print "drawParticles:" + str(particles_new)
#
# for c in range(0, final_position, step):
#     particles[0] = [final_position - c, final_position, 180]
#     particles = UpdateParticlesAfterForward10(particles)
#     particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
#     print "drawParticles:" + str(particles_new)
#     time.sleep(sleep_time)
#
# particles = UpdateParticlesAfterLeft90(particles)
# particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
# print "drawParticles:" + str(particles_new)
#
# for c in range(0, final_position, step):
#     particles[0] = [initial_position, final_position - c, 270]
#     particles = UpdateParticlesAfterForward10(particles)
#     particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
#     print "drawParticles:" + str(particles_new)
#     time.sleep(sleep_time)
#
# particles = UpdateParticlesAfterLeft90(particles)
# particles_new =  [(particles[i][0], particles[i][1], particles[i][2]) for i in range(numberOfParticles)]
# print "drawParticles:" + str(particles_new)
