'''
File: TakeoffAndLand.py
Author: Roger Wang
Description: This file is an example for a basic practice to command the drone to 
             takeoff, hover for 10 seconds and land.           
'''

import FlightController
import sys
import time

# Create a Vehicle object
myCopter = FlightController.Vehicle()

# Connect and initialize the vehicle, enable SITL here
if not myCopter.initialize(simulation = True):
	sys.exit(1)

# Try arming the vehicle
timeoutCounter = 0
while not myCopter.arm():
	timeoutCounter += 1
	if timeoutCounter > 3:
		print "Cannot arm the vehicle after 3 retries."
		sys.exit(1)

# Takeoff
if not myCopter.takeoff(1):
	sys.exit(1)

# Hover for 10 seconds
myCopter.hover(10)

# Land
timeoutCounter = 0
while not myCopter.land():
	timeoutCounter += 1
	if timeoutCounter > 3:
		print "Critical: Cannot land the vehicle after 3 retries."
		sys.exit(1)
		
myCopter.exit()
