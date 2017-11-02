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

motorParamsLeft.pidParameters.k_p = 200
motorParamsLeft.pidParameters.k_i = 0
motorParamsLeft.pidParameters.k_d = 0

motorParamsRight.pidParameters.k_p = 200
motorParamsRight.pidParameters.k_i = 0
motorParamsRight.pidParameters.k_d = 0

interface.setMotorAngleControllerParameters(motors[0],motorParamsLeft)
interface.setMotorAngleControllerParameters(motors[1],motorParamsRight)

# Array of 100 Particles of the form [x, y, th].
particles = [[0,0,0]] * 100

# Array of Weights corresponding to particles.
weights = [0] * 100

def Left90deg():
    print("Turning 90 left")
    angle = 4.85
    interface.increaseMotorAngleReferences(motors, [angle, -angle])
    while not interface.motorAngleReferencesReached(motors): 
	time.sleep(0.1) 
    
def Forward10():
    print("Forward 10")
    distance = 3.0
    interface.increaseMotorAngleReferences(motors, [-distance, -distance])
    while not interface.motorAngleReferencesReached(motors):  
	time.sleep(0.1)    

#logfile = raw_input("Specify logfile: ")
#interface.startLogging("/home/pi/BrickPi/Logfiles/" + logfile)

#Forward10()
#Forward10()
#Forward10()
#Forward10()

#Left90deg()

#Forward10()
#Forward10()
#Forward10()
#Forward10()

#Left90deg()

#Forward10()
#Forward10()
#Forward10()
#Forward10()

#Left90deg()

#Forward10()
#Forward10()
#Forward10()
#Forward10()

#interface.stopLogging()
interface.terminate()
