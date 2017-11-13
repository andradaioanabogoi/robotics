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

num_particles = 100

# Functions to generate some dummy particles data:
def calcX():
    return random.gauss(80,3) + 70*(math.sin(t)); # in cm

def calcY():
    return random.gauss(70,3) + 60*(math.sin(2*t)); # in cm

def calcW():
    return random.random();

def calcTheta():
    return random.randint(0,360);

# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size;    # in cm;
        self.canvas_size = 768;         # in pixels;
        self.margin      = 0.05*map_size;
        self.scale       = self.canvas_size/(map_size+2*self.margin);

    def drawLine(self,line):
        x1 = self.__screenX(line[0]);
        y1 = self.__screenY(line[1]);
        x2 = self.__screenX(line[2]);
        y2 = self.__screenY(line[3]);
        print "drawLine:" + str((x1,y1,x2,y2))

    def drawParticles(self,data):
        display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data];
        print "drawParticles:" + str(display);

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:        
    def __init__(self):
        self.walls = [];

    def add_wall(self,wall):
        self.walls.append(wall);
        
    def get_walls(self):
        return self.walls

    def clear(self):
        self.walls = [];

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall);

# Simple Particles set
class Particles:
        
    def __init__(self):
        self.n = 100;    
        self.data = [];

    def update(self):
        self.data = [[calcX(), calcY(), calcTheta(), calcW()] for i in range(self.n)];
    
    def draw(self):
        canvas.drawParticles(self.data);

canvas = Canvas();    # global canvas we are going to draw on

mymap = Map();
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
mymap.add_wall((0,0,0,168));        # a
mymap.add_wall((0,168,84,168));     # b
mymap.add_wall((84,126,84,210));    # c
mymap.add_wall((84,210,168,210));   # d
mymap.add_wall((168,210,168,84));   # e
mymap.add_wall((168,84,210,84));    # f
mymap.add_wall((210,84,210,0));     # g
mymap.add_wall((210,0,0,0));        # h
mymap.draw();

    
particles = Particles();

# Normalizing the weights of all particles
def normalise_particles(particles):
    #for i in range(0, len(particles.data)):
    #    sum_weights += particles.data[i][3]
    #for i in range(0, len(particles.data)):
    #    particles.data[i][3] = float(particles.data[i][3])/ sum_weights
    sum_weights = 0
    for p in particles.data:
        sum_weights += p[3]
    for p in particles.data:
        p[3] = float(p[3])/sum_weights

def resampling(particles):
    normalise_particles(particles)

    # generate cumulative probability array
    cumulative_prob_array = []
    cumulative_prob = 0
    prev_prob = 0
    
    for p in particles.data:
        prev_prob = cumulative_prob
        cumulative_prob += p[3]
        cumulative_prob_array.append((prev_prob, cumulative_prob, p))
    #print cumulative_prob_array
    
    # choose new particle randomly
    new_particles = []
    for p in particles.data:
        r = random.random()
        for c in cumulative_prob_array:
            # c[0] = lower bound for this particle
            # c[1] = upper bound for this particle
            if r >= c[0] and r < c[1]:
                new_particles.append([c[2][0], c[2][1], c[2][2], 1.0/100])
    particles.data = new_particles

if __name__ == "__main__":
    particles = Particles();
    sum_weights = 0
    particles.data = [[1,2,3,4], [1,0,4,0], [0,1,9,8], [5,4,9,6]]
    #normalise_particles(particles)
    print particles.data
    resampling(particles)
    print particles.data
    
interface.terminate()


