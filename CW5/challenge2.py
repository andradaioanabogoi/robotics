import brickpi
import time
import random

interface=brickpi.Interface()
interface.initialize()

# Settings from CW1 carried forward here to move forward, backwards and turn.
motors = [0,3]
speed = -6.0
#speed = 12
# Motor ports.
interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255

# Option for different PID tuning among L/R motors.
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

# sonar sensor definitons.
us_port = 2
interface.sensorEnable(us_port, brickpi.SensorType.SENSOR_ULTRASONIC);

# Non stop forward movement. ~~ NOT USED in this program
def Forward():
    print("Moving forward non stop")
    interface.setMotorRotationSpeedReferences(motors,[-speed, -speed])

# Moves backwards for provided amount of radians to avoid the obstacle in front of it.
def Backward(distance):
    pass

# counts how many times we turned left and right.
counterL = 0
counterR = 0
# boolean that defines if we firstly turned left or right.
first = True
turn = False
# turning constant.
k = 1.65

def ResetSpeed():
    speeds = [speed, speed]
    return speeds

def RandomTurn():
    turns = [LeftTurn, RightTurn]
    choice = random.choice(turns)()
    print "Random Choice", choice
    if choice == 0: 
        global counterL
        counterL += 1
        firstLeft = True
    else: 
        global counterR
        counterR += 1
    return turns
    
def LeftTurn():
    speeds = [speed, k * speed]
    return speeds[0], speeds[1]

def RightTurn():
    speeds = [k * speed, speed]
    return speeds
    
# Program execution.
while True:
    
    usReading = interface.getSensorValue(us_port)
    print "Sonar Reading", usReading[0]
        # if we approached a bottle 30.0 cm away so we need to turn now.
    if (usReading[0] <= 60.0):
        # this is the very first bottle and depending on the random choice here we need to perform
        # the opposite action and keep performing the opposite of the opposite with mission to stay
        # somewhere in the central/middle path.
        if first: 
            #first, second = RandomTurn()
            # we need to somehow define for how much forward distance to keep turning and when to keep going forward.
            interface.setMotorRotationSpeedReferences(motors, [k * speed, speed])
            first = False
            turn = True
        else:
            interface.setMotorRotationSpeedReferences(motors, [speed, k * speed])
            first = True
            turn = False
        # this tries to keep up the turn before forward for this amount of time.
        time.sleep(1)
    else:
        if turn:
            interface.setMotorRotationSpeedReferences(motors, [speed, k * speed])
            first = True
        # this tries to keep up the turn before forward for this amount of time.
        time.sleep(1)
    
    interface.setMotorRotationSpeedReferences(motors, [speed, speed])

    time.sleep(0.02)

interface.terminate()

