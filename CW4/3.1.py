
  default: printf("Visit appstore.\n");
rt time
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
        self.data = [(calcX(), calcY(), calcTheta(), calcW()) for i in range(self.n)];
    
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

walls = mymap.get_walls()

def calculate_likelihood(x, y, theta, z):
    # Array of ground distances towards to walls
    m = []
    # Updated world coordinates for point of intersection (aka meeting point) with walls in map.
    m_walls = []
    # Returns true if the point of intersection is indeed a wall
    wall_found = False
    # Holds the minimum positive distance to a wall
    current_min = 0
    new_min = 0
    
    # Calculates the ground truth distance between the given particle and a wall.
    for wall in walls:
        m_distance = ((wall[3] - wall[1]) * (wall[0] - x) - (wall[2] - wall[0]) * (wall[1] - y)) / ((wall[3] - wall[1]) * math.cos(theta) - (wall[2] - wall[0]) * math.sin(theta))
        m.append(m_distance)
    # debugging statement
    print "m", m
    
    # calculates the point of intersection with a wall in the world map.
    for m_dist in m:
        x_wall = x + m_dist * math.cos(theta)
        y_wall = y + m_dist * math.sin(theta)
        m_walls.append((x_wall, y_wall))
    print "m_walls", m_walls
    
    for j in range(0, len(walls)):
        # Finds the index of the smallest positive distance from a wall
        while current_min <= 0:
            current_min = random.choice(m)
        min_index = 0    
        for i in range(0, len(m)):
            if j == 0:
                if m[i] < current_min and m[i] > 0: 
                    current_min = m[i]
                    min_index = i
            else:
                if m[i] > current_min: 
                    current_min = m[i]
                    min_index = i
        print "current_min", current_min, "min_index", min_index, "new_min", new_min
        wall_found = check_wall_boundaries(walls[min_index], m_walls[min_index])
        print "wall_found", wall_found
        if wall_found:
            break

def check_wall_boundaries(wall, m_wall):
    # Ax <= Mx and Mx <= Bx and Ay <= My and My <= By
    print "wall", wall, "m_wall", m_wall
    return wall[0] <= m_wall[0] and m_wall[0] <= wall[2] and wall[1] <= m_wall[1] and m_wall[1] <= wall[3]
        
        
    #wallfound =false
    #while wallfound
    #take index of min positive m
    #calculate (x+mcos(theta, y+msin(theta)
    #check wall boundaries
    #if found: wallfound = true
    
calculate_likelihood(69, 72, 135, 240)

#t = 0;
#while True:
#    particles.update();
#    particles.draw();
#    t += 0.05;
#    time.sleep(0.05);
     
    
interface.terminate()

