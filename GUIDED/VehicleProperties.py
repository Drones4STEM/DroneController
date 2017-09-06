'''
File: VehicleProperties.py
Author: Roger Wang
Description: This file is an example for reading property values from the vehicle          
'''

import FlightController
import sys
import time

# Create a Vehicle object
myCopter = FlightController.Vehicle()

# Connect and initialize the vehicle, enable SITL here
if not myCopter.initialize(simulation = True):
	sys.exit(1)

# Print attitude information
print "Roll radians = ", myCopter.attitude.roll
print "Pitch radians = ", myCopter.attitude.pitch
print "Yaw radians = ", myCopter.attitude.yaw

# Print battery information
if myCopter.battery.voltage:
	print "Battery voltage = %s millivolts" % myCopter.battery.voltage
else:
	print "Battery information unavailable"
if myCopter.battery.current:
	print "Battery current = %s milliamperes" % (myCopter.battery.current * 10)
if myCopter.battery.level:
	print "Battery level = ", myCopter.battery.level

# Print channels information
for i in range(1, 9):
	print "Channel %d = %s" % (i, myCopter.channels[str(i)])

# Print rangefinder information
print "Range finder distance = %s meters" % myCopter.rangefinder.distance

myCopter.exit()
