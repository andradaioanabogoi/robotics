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
motorParams.pidParameters.k_p = 250
motorParams.pidParameters.k_i = 225
motorParams.pidParameters.k_d = 0

motorParamsRight = motorParams
motorParamsLeft = motorParams

interface.setMotorAngleControllerParameters(motors[0],motorParamsLeft)
interface.setMotorAngleControllerParameters(motors[1],motorParamsRight)

interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC)

mu = 0.0
sigma = 2.5
sigma_a = 0.01

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
        self.dataTuple = [];

    def update(self):
        self.data = [(calcX(), calcY(), calcTheta(), calcW()) for i in range(self.n)];
        
    def draw(self):
        canvas.drawParticles(self.dataTuple);
        
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

# holds all the likelihoods.
likelihood = []

x = 84
y = 30
th = 0
w = 0.1

#Initial pos
current_position = [x, y, th]
particles.data = [[84, 30, 0, 0.01] for i in range(100)]
print "particles at the beginning: ", particles.data
# print "particles.data[0]: ", particles.data[0]
# print "particles.data[0][0]: ", particles.data[0][0]

# Updates the likelihood of all particles.
def UpdateLikelihood(z):
    global likelihood
    likelihood = []
    for i in range(len(particles.data)):
        #print "counter", i
        likelihood.append(calculate_likelihood(particles.data[i][0], particles.data[i][1], particles.data[i][2], z))
        # multiplies likelihood of a specific particle by its current weight to get the new weight.
        # print "likelihood", likelihood    
        #print "PARTICLES WEIGHTS: ", particles.data[i][3]
        if(likelihood[i] != 0.00000):
            particles.data[i][3] = likelihood[i] * particles.data[i][3]
    print "LIKELIHOOD ARRAY: ", likelihood
    particles.dataTuple = [(particles.data[i][0], particles.data[i][1], particles.data[i][2], particles.data[i][3]) for i in range(100)]

# Updates uncertainty for Forward movement of 20 cm.
def UpdateParticlesAfterForward(dist):
    D = dist
    e = random.gauss(mu, sigma)
    for p in particles.data:
        p[0] = p[0] + (D + e) * math.cos(p[2])
        p[1] = p[1] + (D + e) * math.sin(p[2])
        p[2] = p[2] + random.gauss(mu, sigma_a)
        p[3] = p[3]
    current_position = meanValue()
    particles.dataTuple = [(particles.data[i][0], particles.data[i][1], particles.data[i][2], particles.data[i][3]) for i in range(100)]
    print "current position AFTER FORWARD: ", current_position
    
# Updates uncertainty for Left turn movement.
def UpdateParticlesAfterTurn(angle):
    for p in particles.data:
        p[0] = p[0]
        p[1] = p[1]
        p[2] = p[2] + angle + random.gauss(mu, sigma_a)
        p[3] = p[3]
    current_position = meanValue()
    particles.dataTuple = [(particles.data[i][0], particles.data[i][1], particles.data[i][2], particles.data[i][3]) for i in range(100)]
    print "current position AFTER TURN: ", current_position

def calculate_likelihood(x, y, theta, z):
    # print "INSIDE CALCULATE LIKELIHOOD"
    
    diff_ys = 0
    diff_xs = 0
    
    # Calculates the ground truth distance between the given particle and a wall.
    for wall in walls:
        m_numerator = (wall[3] - wall[1]) * (wall[0] - x) - (wall[2] - wall[0]) * (wall[1] - y)
        m_denominator = (wall[3] - wall[1]) * math.cos(theta) - (wall[2] - wall[0]) * math.sin(theta)
        if m_denominator == 0:
            continue
        else:
            m = m_numerator / m_denominator
        
        x_wall = x + m * math.cos(theta)
        y_wall = y + m * math.sin(theta)
        m_wall = (x_wall, y_wall)

        wall_found = check_wall_boundaries(wall, m_wall)
        if wall_found and m >= 0:
            print "wall_found", wall
            diff_ys = wall[1] - wall[3]
            diff_xs = wall[2] - wall[0]
            break
    
    diff = z - m  
    sd = 2.5  
    e_numerator = -math.pow(diff, 2)
    e_denominator = 2 * math.pow(sd, 2)
    likelihood = math.pow(math.e, e_numerator/e_denominator)
    
    # Implements the more sophisticated check about how big is the incidence angle that the measure is taken from and if this is too big therefore not reliable we may skip the update.
    numerator = math.cos(diff_ys) + math.sin(diff_xs)
    denominator = math.sqrt(math.pow(diff_ys, 2) + math.pow(diff_xs, 2))
    if denominator != 0:
        b = math.acos(numerator/denominator)
        #if incidence_angle_too_big(b):
        #    print "Inidence angle too big -> likelihood set to one"
        #   likelihood = 1
    
    # print "likelihood", likelihood,
    return likelihood


def meanValue():
    mean = [0, 0, 0] 
    for p in particles.data:
        mean[0] += p[3] * p[0]
        mean[1] += p[3] * p[1]
        mean[2] += p[3] * p[2]
    # print "Mean: ", mean
    return mean
    

def check_wall_boundaries(wall, m_wall):
    # Ax <= Mx and Mx <= Bx and Ay <= My and My <= By
    min_wall_x = wall[0] if wall[0] < wall[2] else wall[2]
    max_wall_x = wall[2] if wall[2] > wall[0] else wall[0]
    min_wall_y = wall[1] if wall[1] < wall[3] else wall[3]
    max_wall_y = wall[3] if wall[3] > wall[1] else wall[1]
    return (min_wall_x <= m_wall[0] and m_wall[0] <= max_wall_x) and (min_wall_y <= m_wall[1] and m_wall[1] <= max_wall_y)

# Normalizing the weights of all particles
def normalise_particles(particles):
    #for i in range(0, len(particles.data)):
    #    sum_weights += particles.data[i][3]
    #for i in range(0, len(particles.data)):
    #    particles.data[i][3] = float(particles.data[i][3])/ sum_weights
    sum_weights = 0.0
    for p in particles.data:
        sum_weights += p[3]
    for p in particles.data:
        p[3] = p[3]/sum_weights
    print "NORMALIZED PARTICLES: ", particles.data

def resampling(particles):
    #normalise_particles(particles)
    #print (particles.data)

    # generate cumulative probability array
    cumulative_prob_array = []
    cumulative_prob = 0
    prev_prob = 0
    
    for p in particles.data:
        prev_prob = cumulative_prob
        cumulative_prob += p[3]
        cumulative_prob_array.append((prev_prob, cumulative_prob, p))
    #print "CUMULATIVE ARRAY: ", cumulative_prob_array
    
    # choose new particle randomly
    new_particles = []
    for p in particles.data:
        r = random.uniform(0, 1)
        #print r
        for c in cumulative_prob_array:
            # c[0] = lower bound for this particle
            # c[1] = upper bound for this particle
            if r >= c[0] and r < c[1]:
                new_particles.append([c[2][0], c[2][1], c[2][2], 0.01])
    particles.data = new_particles
    print "RESAMPLED particles: ", particles.data
        
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
    b1 = b
    if b > math.pi:
         b1 = b - 2*math.pi
    #translates the angle to turn to the wheel angle
    angle = (b1*3.777*2)/math.pi 
    print "angle", angle
    # print "Global x, y, th: " + str(x) + " " + str(y) + " " +  str(th)
    TurndegDL(angle)
    UpdateParticlesAfterTurn(b)
    particles.draw();
    time.sleep(0.05);
    z = get_sonar_reading()
    UpdateLikelihood(z)
    particles.draw();
    time.sleep(0.05);
    print "particles.data after updating turn: ", particles.data
    normalise_particles(particles)
    resampling(particles)

    while dist > 0.0:
        print "TOTAL DISTANCE TO DESTINATION IS: ", dist
        min_dist = min(20.0, dist)
        ForwardCm(min_dist)
        UpdateParticlesAfterForward(min_dist)
        particles.draw();
        time.sleep(0.05);
        z = get_sonar_reading()
        UpdateLikelihood(z)
        particles.draw();
        time.sleep(0.05);
        normalise_particles(particles)
        resampling(particles)
        dist = dist - min_dist
    
    '''number_of_twenties = dist // 20
    remainder = dist % 20
    
    #for 20, recalibrate loop
    for x in range(0, int(number_of_twenties)):
        ForwardCm(20)
        # particles.updateAfterForward(20)
        UpdateParticlesAfterForward(20)
        z = get_sonar_reading()
        UpdateLikelihood(z)
        normalise_particles(particles)
        resampling(particles)
        time.sleep(1)     
    
    if remainder > 0.0:
        # remainder forward distance and MCL algorithm execution.
        ForwardCm(remainder)
        UpdateParticlesAfterForward(remainder)
        z = get_sonar_reading()
        UpdateLikelihood(z)
        normalise_particles(particles)
        resampling(particles)'''
    time.sleep(1)
    current_position = meanValue()
    #current_position = particles.data[0]
    print "current_position", current_position
    
#TurndegDL(3.777)
#time.sleep(1.0)
#TurndegDL(-3.9)
#radius for 20cm 
r = 3.35

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
                # print "sonar reading", usReading[0]
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

'''t = 0;
while True:
    print "Current position", current_position
    print "\n /// NAVIGATING TO WAYPOINT 2 /// \n"
    navigateToWaypoint(180,30)
    particles.update();
    particles.draw();
    t += 0.05;
    time.sleep(0.05);'''

interface.terminate()

