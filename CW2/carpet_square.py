import brickpi
import time

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

motorParamsLeft.pidParameters.k_p = 100
motorParamsLeft.pidParameters.k_i = 25
motorParamsLeft.pidParameters.k_d = 25

motorParamsRight.pidParameters.k_p = 115
motorParamsRight.pidParameters.k_i = 25
motorParamsRight.pidParameters.k_d = 25

interface.setMotorAngleControllerParameters(motors[0],motorParamsLeft)
interface.setMotorAngleControllerParameters(motors[1],motorParamsRight)

def Left90deg():
    print("Turning 90 left")
    THRESHOLD = 3
    angle = 4.85
    startTime = time.time()
    interface.increaseMotorAngleReferences(motors, [angle, -angle])
    while interface.motorAngleReferencesReached: 
        timeNow = time.time()
	if timeNow - startTime > THRESHOLD:
		print "Done with turning 90 left -> Ready to execute next move!"
		break
    
def Right90deg():
    print("Turning 90 right")
    THRESHOLD = 3
    angle = 4.85
    startTime = time.time()
    interface.increaseMotorAngleReferences(motors, [-angle, angle])
    while interface.motorAngleReferencesReached:
        timeNow = time.time()
        if timeNow - startTime > THRESHOLD:
                print "Done with turning 90 right -> Ready to execute next move!"
                break

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
    print("Backwards 40")
    THRESHOLD = 3
    distance = 11.75
    startTime = time.time()
    interface.increaseMotorAngleReferences(motors, [distance, distance])
    while interface.motorAngleReferencesReached:
        timeNow = time.time()
        if timeNow - startTime > THRESHOLD:
                print "Done with backwards 40 -> Ready to execute next move!"
                break    

#logfile = raw_input("Specify logfile: ")
#interface.startLogging("/home/pi/BrickPi/Logfiles/" + logfile)

Right90deg()
#Forward40()
#Left90deg()
#Backwards40()

#interface.stopLogging()
interface.terminate()
