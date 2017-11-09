import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

port = 2 # port which ultrasoic sensor is plugged in to

interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC);

sonar_values = [] * 100

def generate_report(trial):

	sonar_list = []

	for t in sonar_values:
		sonar_list.append(t[0])

	print "\n---------- SONAR REPORT ----------\n"
	print "Trial distance " + str(trial) + ": " + str(sonar_list.count(trial))
	print "+1 :" + str(sonar_list.count(trial+1))
	print "+2 :" + str(sonar_list.count(trial+2))
	print "+3 :" + str(sonar_list.count(trial+3))
	print "+4 :" + str(sonar_list.count(trial+4))
	print "+5 :" + str(sonar_list.count(trial+5))
	print "-1 :" + str(sonar_list.count(trial-1))
	print "-2 :" + str(sonar_list.count(trial-2))
	print "-3 :" + str(sonar_list.count(trial-3))
	print "-4 :" + str(sonar_list.count(trial-4))
	print "-5 :" + str(sonar_list.count(trial-5))
	print "255 :" + str(sonar_list.count(255.0))

while True:
	
	trial = input("What distance are you measuring? -> ")

	# get 100 measurements
	for i in range(0, 100):

        	usReading = interface.getSensorValue(port)
        	if usReading :
                	sonar_values.append(usReading)
        	else:
			sonar_values.append("N/M")
        	time.sleep(0.05)

	generate_report(trial)	
	break

interface.terminate()

