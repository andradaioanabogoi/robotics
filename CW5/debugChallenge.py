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

# touch ports 
L_touch_port = 0
R_touch_port = 3

interface.sensorEnable(L_touch_port, brickpi.SensorType.SENSOR_TOUCH);
interface.sensorEnable(R_touch_port, brickpi.SensorType.SENSOR_TOUCH);

# boolean that defines if we firstly turned right.
right = random.choice([True, False])
# boolean that defines if we have turned or not.
left = random.choice([True, False])
# turning constant.
k = 1.65
    
# Program execution.
while True:
    
    usReading = interface.getSensorValue(us_port)
    L_touched = interface.getSensorValue(L_touch_port)
    R_touched = interface.getSensorValue(R_touch_port)

       
    if (usReading[0] <= 40.0):
        if (not right and left) or (left and left): 
            print "Turn right"
            interface.setMotorRotationSpeedReferences(motors, [k * speed, speed])
            # right = True
            # left = False
            time.sleep(1.25)
        
            print "Already right so turn left"
            interface.setMotorRotationSpeedReferences(motors, [speed, k * speed])
            left = random.choice([True, False])
            right = not right
            time.sleep(1.25)
        
        elif (not left and right) or (right and right):
            print "Turn left"
            interface.setMotorRotationSpeedReferences(motors, [speed, k * speed])
            # left = True
            # right = False
            time.sleep(1.25)
            
            print "Already left so turn right"
            interface.setMotorRotationSpeedReferences(motors, [k * speed, speed])
            left = not left
            right = random.choice([True, False])
            time.sleep(1.25)
            
    if L_touched[0] or R_touched[0]:
        d = 3
        interface.increaseMotorAngleReferences(motors, [d, d])
        while not interface.motorAngleReferencesReached(motors):  
            time.sleep(0.1)
        if L_touched[0] and not R_touched[0]:
            interface.increaseMotorAngleReferences(motors, [-d/2, d/2])
            while not interface.motorAngleReferencesReached(motors): 
	            time.sleep(0.1)
        elif R_touched[0] and not L_touched[0]:
            interface.increaseMotorAngleReferences(motors, [d/2, -d/2])
            while not interface.motorAngleReferencesReached(motors): 
	            time.sleep(0.1)
        else:
            interface.increaseMotorAngleReferences(motors, [d/2, -d/2])
            while not interface.motorAngleReferencesReached(motors): 
	            time.sleep(0.1)
    time.sleep(0.5)
        
    interface.setMotorRotationSpeedReferences(motors, [speed, speed])
    
    time.sleep(0.02)

interface.terminate()

