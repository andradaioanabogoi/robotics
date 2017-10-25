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
#    while not interface.motorAngleReferencesReached(motors) :
 #       motorAngles = interface.getMotorAngles(motors)
  #          if motorAngles :
                
   #         time.sleep(0.1)
    speed = motorParams.maxRotationAcceleration
    angle = 3.78839

    interface.increaseMotorAngleReferences(motors, [angle, -angle])

#    interface.setMotorAngleControllerParameters(motors[0],motorParams)
 #   interface.setMotorAngleControllerParameters(motors[1],motorParams)

    interface.setMotorRotationSpeedReferences(motors,[speed,-speed])

    
def Right90deg():
    pass

#while True:
#	angle = float(input("Enter a angle to rotate (in radians): "))


#	interface.increaseMotorAngleReferences(motors,[angle,angle])

#	while not interface.motorAngleReferencesReached(motors) :
#		motorAngles = interface.getMotorAngles(motors)
#		if motorAngles :
#			print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
#		time.sleep(0.1)

#	print "Destination reached!


while True:
    Left90deg()
    time.sleep(10000)

interface.terminate()
