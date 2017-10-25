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
motorParams.pidParameters.k_p = 350.0
motorParams.pidParameters.k_i = 300.0
motorParams.pidParameters.k_d = 50.0

motorParamsRight = motorParams
motorParamsLeft = motorParams

#motorParamsRight.pidParameters.k_p = 250
#motorParamsLeft.pidParameters.k_p = 210
#motorParamsRight.pidParameters.k_d = -10.0
#motorParamsLeft.pidParameters.k_d = 0

interface.setMotorAngleControllerParameters(motors[0],motorParamsLeft)
interface.setMotorAngleControllerParameters(motors[1],motorParamsRight)
#starting to log for tuning 
logfile = raw_input("Specify logfile: ")
interface.startLogging("/home/pi/BrickPi/Logfiles/" + logfile)
while True:
	angle = float(input("Enter a angle to rotate (in radians): "))


	interface.increaseMotorAngleReferences(motors,[angle,angle])

	while not interface.motorAngleReferencesReached(motors) :
		motorAngles = interface.getMotorAngles(motors)
		if motorAngles :
			print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
		time.sleep(0.1)

	print "Destination reached!"
#stop to log for tuning	
interface.stopLogging()
interface.terminate()
