'''
File: Velocity&YawCtrl.py
Author: Roger Wang
Description: This file is an example for a basic practice to command the drone to 
             takeoff, control its velocity, yaw orientation and land.           
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

# Hover for 5 seconds
myCopter.hover(5)

#####################################################################
# 
#                       Velocity Control Examples
#
# Go eastward at 0.2m/s for 5 seconds (absolute velocity)
print "Going eastward at 0.2m/s for 5s"
myCopter.send_nav_velocity(0, 0.2, 0, relative = False)
time.sleep(5)

# Hover for 5 seconds
print "Hovering"
myCopter.hover(5)

# Go forward at 0.2m/s for 5 seconds (relative velocity)
print "Going forward at 0.2m/s for 5s"
myCopter.send_nav_velocity(0.2, 0, 0)
time.sleep(5)

# Hover for 5 seconds
print "Hovering"
myCopter.hover(5)


#####################################################################
#
#                        Yaw Control Examples
#
# Change yaw direction to (0, north) and wait for 5 seconds (absolute heading)
print "Change the vehicle direction to north"
myCopter.condition_yaw(0, relative = False)
time.sleep(5)

# Change yaw direction by 60 degrees, counter clock wise (relative heading)
print "Change yaw direction by 60 degrees, counter clock wise"
myCopter.condition_yaw(60, relative = True, clock_wise = False)
time.sleep(5)

# Hover for 5 seconds
print "Hovering"
myCopter.hover(5)

#####################################################################
#
#                       Position Control Examples
#
# Go forward for 1m
print "Go forward for 1m"
myCopter.goto(1, 0, 0, relative = True)
time.sleep(10)

# Go eastward for 1m
print "Go eastward for 1m"
myCopter.goto(0, 1, 0, relative = False)
time.sleep(10)

# Hover for 5 seconds
print "Hovering"
myCopter.hover(5)

# Land
timeoutCounter = 0
while not myCopter.land():
	timeoutCounter += 1
	if timeoutCounter > 3:
		print "Critical: Cannot land the vehicle after 3 retries."
		sys.exit(1)
		
myCopter.exit()
