import brickpi
import time

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

motorParamsLeft.pidParameters.k_p = 340
motorParamsLeft.pidParameters.k_i = 300
motorParamsLeft.pidParameters.k_d = 300

motorParamsRight.pidParameters.k_p = 340 
motorParamsRight.pidParameters.k_i = 300
motorParamsRight.pidParameters.k_d = 300

interface.setMotorAngleControllerParameters(motors[0],motorParamsLeft)
interface.setMotorAngleControllerParameters(motors[1],motorParamsRight)

# sonar sensor definitons.
us_port = 1
interface.sensorEnable(us_port, brickpi.SensorType.SENSOR_ULTRASONIC);

# Non stop forward movement.
def Forward():
    print("Moving forward non stop")
    interface.setMotorRotationSpeedReferences(motors,[-speed,-speed])    
    while True:
        time.sleep(1)

# Moves backwards for provided amount of radians to avoid the obstacle in front of it.
def Backward(distance):
    pass
                                
# Program execution.
while True:
	usReading = interface.getSensorValue(port)

    while usReading != 30:
        error = usReading - 30
        speed = -error * speed
        interface.setMotorRotationSpeedReferences(motors,[-speed,-speed])

        # maybe we need to use the function motorRotationSpeedReferenceReached(...) somewhere.
        
	time.sleep(0.05)

interface.terminate()