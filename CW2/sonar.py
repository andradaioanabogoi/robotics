import brickpi
import time
from numpy import median

interface=brickpi.Interface()
interface.initialize()

# Settings from CW1 carried forward here to move forward, backwards and turn.
motors = [0,3]
speed = 6.0

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

motorParamsLeft.pidParameters.k_p = 100
motorParamsLeft.pidParameters.k_i = 25
motorParamsLeft.pidParameters.k_d = 25

motorParamsRight.pidParameters.k_p = 115 
motorParamsRight.pidParameters.k_i = 25
motorParamsRight.pidParameters.k_d = 25

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

# Improves measures but taking median of last few measures
def medianSonar():
   MEASURES = 8
   sonar = [0]*MEASURES
   i = 0
   while i < MEASURES:
     sonar[i] = interface.getSensorValue(us_port)
     i += 1
     time.sleep(0.01)  
   return int(median(sonar))

# Program execution.
while True:
    
    usReading = interface.getSensorValue(us_port)
   
    # otherwise we use the suggested median measurement that follows.
    # usReading = medianSonar()
    
    # used for debugging
    print "usReading ZERO: " + str(usReading[0])
    print "Error ZERO: " + str(usReading[0]-35.0)

    while usReading[0] != 35.0:
        error = usReading[0] - 35.0
	print "usReading ONE: " + str(usReading[0])
        print "Error ONE: " + str(error)

	# here we need to use velocity control and set the velocity demands of both wheels to be proportional to the
	# error between the actual and the desired distance using prop control with a single gain value.
	speed = speed * -error

	while interface.motorRotationSpeedReferenceReached:
            interface.setMotorRotationSpeedReferences(motors, [speed, speed])

        # maybe we need to use the function motorRotationSpeedReferenceReached(...) somewhere.
        
	time.sleep(0.5)

interface.terminate()
