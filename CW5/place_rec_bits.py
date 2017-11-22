#!/usr/bin/env python
# By Jacek Zienkiewicz and Andrew Davison, Imperial College London, 2014
# Based on original C code by Adrien Angeli, 2009

import random
import os
import brickpi
import time
import math

interface=brickpi.Interface()
interface.initialize()

motors = [1]
port = 2  # Used for sonar measurement 

interface.motorEnable(motors[0])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255

motorParams.pidParameters.k_p = 250
motorParams.pidParameters.k_i = 225
motorParams.pidParameters.k_d = 0

interface.setMotorAngleControllerParameters(motors[0], motorParams)

interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC)

# Location signature class: stores a signature characterizing one location
class LocationSignature:
    def __init__(self, no_bins = 360):
        self.sig = [0] * no_bins
        
    def print_signature(self):
        for i in range(len(self.sig)):
            print self.sig[i]

# --------------------- File management class ---------------
class SignatureContainer():
    def __init__(self, size = 5):
        self.size      = size; # max number of signatures that can be stored
        self.filenames = [];
        
        # Fills the filenames variable with names like loc_%%.dat 
        # where %% are 2 digits (00, 01, 02...) indicating the location number. 
        for i in range(self.size):
            self.filenames.append('loc_{0:02d}.dat'.format(i))

    # Get the index of a filename for the new signature. If all filenames are 
    # used, it returns -1;
    def get_free_index(self):
        n = 0
        while n < self.size:
            if (os.path.isfile(self.filenames[n]) == False):
                break
            n += 1
            
        if (n >= self.size):
            return -1;
        else:    
            return n;
 
    # Delete all loc_%%.dat files
    def delete_loc_files(self):
        print "STATUS:  All signature files removed."
        for n in range(self.size):
            if os.path.isfile(self.filenames[n]):
                os.remove(self.filenames[n])
            
    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def save(self, signature, index):
        filename = self.filenames[index]
        if os.path.isfile(filename):
            os.remove(filename)
            
        f = open(filename, 'w')

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close();

    # Read signature file identified by index. If the file doesn't exist
    # it returns an empty signature.
    def read(self, index):
        ls = LocationSignature()
        filename = self.filenames[index]
        if os.path.isfile(filename):
            f = open(filename, 'r')
            for i in range(len(ls.sig)):
                s = f.readline()
                if (s != ''):
                    ls.sig[i] = int(s)
            f.close();
        else:
            print "WARNING: Signature does not exist."
        
        return ls

# Rotates sonar one degree
def rotate_sonar_1d():
    # 1 degree in radians is 0.0174533
    interface.increaseMotorAngleReferences(motors, [-0.0174533])
    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.05)
    
# Returns a sonar measurement    
def get_sonar_reading():
    while True:
        for i in range(1):
            usReading = interface.getSensorValue(port)
            if usReading:
                # print "sonar reading", usReading[0]
                return usReading[0]
        break    
    
    
# FILL IN: spin robot or sonar to capture a signature and store it in ls
def characterize_location(ls):
    print "TODO:    You should implement the function that captures a signature."
    for i in range(len(ls.sig)):
        # make mounted sonar to rotate in each degree.
        rotate_sonar_1d()
        ls.sig[i] = get_sonar_reading()
    print ls.sig
    print "length of sig array", len(ls.sig)

# FILL IN: compare two signatures
def compare_signatures(ls1, ls2):
    dist = 0
    print "TODO:    You should implement the function that compares two signatures."
    # implements the correlation test
    # assumes that ls1 is the new observation and ls2 is the already saved following what is done in
    # recognise location function.
    for i in range(0, len(ls1.sig)):
        diff_sq = math.pow(ls1.sig[i] - ls2.sig[i], 2)
        dist += diff_sq
    print "distance:", dist
    return dist
    # after returning the distance it should pick the minimum distance found so a good way to do this
    # is to place distances in an array of distances that matches the known waypoints that the lecturer wants
    # us to test on which are the first 5 waypoints, so distances[0] will be the 1st waypoint and so on.
    # I believe the place that allows is the recognize_location() function.

# This function characterizes the current location, and stores the obtained 
# signature into the next available file.
def learn_location():
    ls = LocationSignature()
    characterize_location(ls)
    idx = signatures.get_free_index();
    if (idx == -1): # run out of signature files
        print "\nWARNING:"
        print "No signature file is available. NOTHING NEW will be learned and stored."
        print "Please remove some loc_%%.dat files.\n"
        return
    
    signatures.save(ls,idx)
    print "STATUS:  Location " + str(idx) + " learned and saved."

# This function tries to recognize the current location.
# 1.   Characterize current location
# 2.   For every learned locations
# 2.1. Read signature of learned location from file
# 2.2. Compare signature to signature coming from actual characterization
# 3.   Retain the learned location whose minimum distance with
#      actual characterization is the smallest.
# 4.   Display the index of the recognized location on the screen
def recognize_location():
    ls_obs = LocationSignature();
    characterize_location(ls_obs);

    # holds the distances of the observed location to each known location.
    distances_diff = [] * signatures.size()
    
    # FILL IN: COMPARE ls_read with ls_obs and find the best match
    for idx in range(signatures.size):
        print "STATUS:  Comparing signature " + str(idx) + " with the observed signature."
        ls_read = signatures.read(idx);
        dist    = compare_signatures(ls_obs, ls_read)
        distances_diff.append(dist)
    # implementing what is discussed above after return dist in compare_signatures function.
    min_distance_index = distances_diff.index(min(distances_diff))
    rec_location = signatures.read(min_distance_index)
    

# Prior to starting learning the locations, it should delete files from previous
# learning either manually or by calling signatures.delete_loc_files(). 
# Then, either learn a location, until all the locations are learned, or try to
# recognize one of them, if locations have already been learned.

#signatures = SignatureContainer(5);
#signatures.delete_loc_files()

#learn_location();
#recognize_location();

# testing characterize location incrementally, result: PASSED
# ls = LocationSignature()
# characterize_location(ls)

# testing compare_signatures incrementally, result: PASSED
ls1 = LocationSignature()
ls2 = LocationSignature()
compare_signatures(ls1, ls2)

# testing recognize_location function, result: PENDING


