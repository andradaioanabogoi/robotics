import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 340
motorParams.pidParameters.k_i = 300
motorParams.pidParameters.k_d = 300

motorParamsRight = motorParams
motorParamsLeft = motorParams

interface.setMotorAngleControllerParameters(motors[0],motorParamsLeft)
interface.setMotorAngleControllerParameters(motors[1],motorParamsRight)

def Left90deg():
    print("turning 90 left")
    THRESHOLD = 1.05
    angle = 6.65
    startTime = time.time()
    interface.increaseMotorAngleReferences(motors, [angle, -angle])
    while interface.motorRotationSpeedReferenceReached:
        timeNow = time.time()
	if timeNow - startTime > THRESHOLD:
		print "stopping because we did not reach"
		break
    
def Right90deg():
    print("turning 90 right")
    THRESHOLD = 1.05
    angle = 6.06
    startTime = time.time()
    interface.increaseMotorAngleReferences(motors, [-angle, angle])
    while interface.motorRotationSpeedReferenceReached:
        timeNow = time.time()
        if timeNow - startTime > THRESHOLD:
                print "stopping because we did not reach"
                break

def Forward40():
    print("forward 40")
    THRESHOLD = 4
    distance = 11.75
    startTime = time.time()
    interface.increaseMotorAngleReferences(motors, [-distance, -distance])
    while interface.motorRotationSpeedReferenceReached:
        timeNow = time.time()
        if timeNow - startTime > THRESHOLD:
                print "stopping because we did not reach"
                break

def Backwards40():
    pass    

Forward40()

interface.terminate()
