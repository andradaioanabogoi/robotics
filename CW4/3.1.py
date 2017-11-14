import time
import sys
import random
import math
import brickpi

interface=brickpi.Interface()
interface.initialize()

motors = [0,3]
port = 2 # port which ultrasoic sensor is plugged in to


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

interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC)

mu = 0.0
sigma = 2.5
sigma_a = 0.01

# Functions to generate some dummy particles data:
def calcX():
    return random.gauss(80,3) + 70*(math.sin(t)); # in cm

def calcY():
    return random.gauss(70,3) + 60*(math.sin(2*t)); # in cm

def calcW():
    return random.random;

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
        self.data = []; # data contents updated in class initialisation.

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

particles_b_angles = []

x = 84
y = 30
th = 0
w = 0.1

#Initial pos
current_position = [x, y, th]
particles.data = [[84, 30, 0, 0.01] for i in range(100)]
# print "particles.data[0]: ", particles.data[0]
# print "particles.data[0][0]: ", particles.data[0][0]

# Updates uncertainty for Forward movement of 20 cm.
def UpdateParticlesAfterForward(dist):
    D = dist * canvas.map_size
    e = random.gauss(mu, sigma)
    for p in particles.data:
        p[0] = p[0] + (D + e) * math.cos(p[2])
        p[1] = p[1] + (D + e) * math.sin(p[2])
        p[2] = p[2] + random.gauss(mu, sigma_a)

# Updates uncertainty for Left 90 movement.
def UpdateParticlesAfterTurn(angle):
    for p in particles.data:
        p[0] = p[0]
        p[1] = p[1]
        p[2] = p[2] + angle + random.gauss(mu, sigma_a)

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
        numerator = (wall[3] - wall[1]) * (wall[0] - x) - (wall[2] - wall[0]) * (wall[1] - y)
        denominator = (wall[3] - wall[1]) * math.cos(theta) - (wall[2] - wall[0]) * math.sin(theta)
        if denominator == 0:
            m.append(-300)
        else:
            m_distance = numerator / denominator
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
        # Finding smallest positive distance from a wall.
        while current_min <= 0:
            current_min = random.choice(m)
        min_index = 0
        # It does so by iterating over all 8 walls.
        for i in range(0, len(m)):
            if j == 0:
                if m[i] <= current_min and m[i] > 0: 
                    current_min = m[i]
                    min_index = i
            # Gets the second/third/etc positive distance from a wall in case first one is not a meeting point with a real world.
            else:
                if m[i] > current_min: 
                    current_min = m[i]
                    min_index = i
        # debugging statements.
        print "current_min", current_min, "min_index", min_index
        wall_found = check_wall_boundaries(walls[min_index], m_walls[min_index])
        print "wall_found", wall_found
        if wall_found:
            break
    
    # Calculation of the likelihood value.
    print "z, m_index, m[m_index]", z, min_index, m[min_index]
    diff = z - m[min_index]    
    sd = 2.5  
    e_numerator = -math.pow(diff, 2)
    e_denominator = 2 * math.pow(sd, 2)
    likelihood = math.pow(math.e, e_numerator/e_denominator)
    
    # Implements the more sophisticated check about how big is the incidence angle that the measure is taken from and if this is too big therefore not reliable we may skip the update.
    diff_ys = walls[min_index][1] - walls[min_index][3]
    diff_xs = walls[min_index][2] - walls[min_index][0]
    numerator = math.cos(diff_ys) + math.sin(diff_xs)
    denominator = math.sqrt(math.pow(diff_ys, 2) + math.pow(diff_xs, 2))
    b = math.acos(numerator/denominator)
    
    # holds particle's opinion if incidence angle is too big.
    b_too_big = incidence_angle_too_big(b)
    # adds this opinion on a global array of particle angles.
    particles_b_angles.append(b_too_big)
    
    print "likelihood", likelihood
    return likelihood


def meanValue():
    mean = [0,0,0]
    # it was not getting inside here because there were not data inside here, IF WE
    # GENERATE FOR EXAMPLE: particles.data = [x,y,th,w] as I did above then is ok.
    # Currently I am not sure if we need to set mean to 0,0,0 it makes  sense to me.
    for p in particles.data:
        #mean += weights[i]*particles[i]
        mean[0] += p[3] * p[0]
        mean[1] += p[3] * p[1]
        mean[2] += p[3] * p[2]
        # print "p[0]: ", p[0]
        # print "p[1]: ", p[1]
        # print "p[2]: ", p[2]
        # print "p[3]: ", p[3]
    print "Mean: ", mean
    return mean
    

def check_wall_boundaries(wall, m_wall):
    # Ax <= Mx and Mx <= Bx and Ay <= My and My <= By
    print "wall[0] <= m_wall[0]", "m_wall[0] <= wall[2])", "(wall[1] <= m_wall[1]", "m_wall[1] <= wall[3]", wall[0] <= m_wall[0], m_wall[0] <= wall[2], wall[1] <= m_wall[1], m_wall[1] <= wall[3]
    print "wall", wall, "m_wall", m_wall
    min_wall_x = wall[0] if wall[0] < wall[2] else wall[2]
    max_wall_x = wall[2] if wall[2] > wall[0] else wall[0]
    min_wall_y = wall[1] if wall[1] < wall[3] else wall[3]
    max_wall_y = wall[3] if wall[3] > wall[1] else wall[1]
    return (min_wall_x <= m_wall[0] and m_wall[0] <= max_wall_x) and (min_wall_y <= m_wall[1] and m_wall[1] <= max_wall_y)
        
        
def incidence_angle_too_big(angle):
    # returns true if incidence angle is bigger than 10.0 degrees (0.174533 radians) which affects reliability.
    return angle > 0.174533
        
# Nicolay waypoint function
def navigateToWaypoint(wx, wy):
    global current_position
    dx = wx - current_position[0]
    dy = wy - current_position[1]
    th = current_position[2]
    
    print "dx, dy: " + str(dx) +  " " + str(dy)
    
    #calculates the angle of a new vector
    a = (math.atan2(dy,dx) + 2*math.pi) % (2*math.pi)

    b = (a - th + 2*math.pi) % (2*math.pi)
    #arbitary unit distance proportional to coordinate system
    dist = math.sqrt(math.pow(dx,2) + math.pow(dy,2))
    #determines if its a right or left rotation
    if b > math.pi:
         b = b - 2*math.pi
    #translates the angle to turn to the wheel angle
    angle = (b*3.777*2)/math.pi 
    # print "Global x, y, th: " + str(x) + " " + str(y) + " " +  str(th)
    #moves the robot
    TurndegDL(angle)
    #TurndegDL(3.777)
    UpdateParticlesAfterTurn(angle)
    
    number_of_twenties = dist // 20
    remainder = dist % 20
    
    #for 20, recalibrate loop
    for x in range(0, int(number_of_twenties)):
        ForwardCm(20)
        UpdateParticlesAfterForward(20)
        print "angle", a
        z = get_sonar_reading()
        print "z", z
        calculate_likelihood(180, 30, a, z)
        time.sleep(1)     
    
    #remainder move, recalibrate
    ForwardCm(remainder)
    UpdateParticlesAfterForward(remainder)
    print "angle", a
    z = get_sonar_reading()
    print "z", z
    likelihod = calculate_likelihood(180, 30, a, z)
    time.sleep(1)
    current_position = meanValue()
    print "current_position", current_position
    
#TurndegDL(3.777)
#time.sleep(1.0)
#TurndegDL(-3.9)
#radius for 20cm 
r = 3.23

def ForwardCm(d):
    x = d/r
    interface.increaseMotorAngleReferences(motors, [-x, -x])
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
    
def get_sonar_reading():
    while True:
        for i in range(1):
            usReading = interface.getSensorValue(port)
            if usReading:
                print "sonar reading", usReading[0]
                return usReading[0]
        break


    
### Main Program Execution ###    

# TESTING PLAN
# 1) Navigate to the first waypoint that the tutorial sheet is giving.
# 2) Test the calculateLikelyhoold function by observing if the robot is giving the correct wall and distance from it.

print "Current position", current_position

print "\n /// NAVIGATING TO WAYPOINT 2 /// \n"
navigateToWaypoint(180,30)

print "Current position", current_position

print "\n /// NAVIGATING TO WAYPOINT 3 /// \n"
navigateToWaypoint(180,54)

print "Current position", current_position

print "\n /// NAVIGATING TO WAYPOINT 4 /// \n"
navigateToWaypoint(138,54)

print "Current position", current_position

print "\n /// NAVIGATING TO WAYPOINT 5 /// \n"
navigateToWaypoint(138,168)

print "Current position", current_position

print "\n /// NAVIGATING TO WAYPOINT 6 /// \n"
navigateToWaypoint(114,168)

print "Current position", current_position

print "\n /// NAVIGATING TO WAYPOINT 7 /// \n"
navigateToWaypoint(114,84)

print "Current position", current_position

print "\n /// NAVIGATING TO WAYPOINT 8 /// \n"
navigateToWaypoint(84,84)

print "Current position", current_position

print "\n /// NAVIGATING TO WAYPOINT 9 /// \n"
navigateToWaypoint(84,30)

print "Current position", current_position

t = 0;
while True:
    particles.update();
    particles.draw();
    t += 0.05;
    time.sleep(0.05);
    
interface.terminate()

