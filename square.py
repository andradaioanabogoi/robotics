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
    print("Turning 90 left")
    THRESHOLD = 3
    angle = 3.75
    startTime = time.time()
    interface.increaseMotorAngleReferences(motors, [angle, -angle])
    while interface.motorAngleReferencesReached: 
        timeNow = time.time()
	if timeNow - startTime > THRESHOLD:
		print "Done with turning 90 left -> Ready to execute next move!"
		break
    
def Right90deg():
    pass

def Forward40():
    print("Forward 40")
    THRESHOLD = 3
    distance = 11.75
    startTime = time.time()
    interface.increaseMotorAngleReferences(motors, [-distance, -distance])
    while interface.motorAngleReferencesReached:
        timeNow = time.time()
        if timeNow - startTime > THRESHOLD:
                print "Done with forward 40 -> Ready to execute next move!"
                break

def Backwards40():
    pass    

Forward40()
Left90deg()
Forward40()
Left90deg()
Forward40()
Left90deg()
Forward40()

interface.terminate()
