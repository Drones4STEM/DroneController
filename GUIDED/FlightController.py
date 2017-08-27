'''
File: FlightController.py
Author: Roger Wang
Description: This File contains controlling methods for drones
             implemented with Optical Flow sensor, range finder and
             a companion computer. This script is designed for indoor 
			 missions.
'''

import dronekit
import time

# Vehicle initialization check time in seconds
INITIALIZE_CHECK_TIME = 1
# Initialization timeout in seconds
INITIALIZE_TIMEOUT = 60
# Mode switching check time in seconds
SWITCH_MODE_CHECK_TIME = 0.5
# Mode switching timeout in seconds
SWITCH_MODE_TIMEOUT = 2
# Arming check time in seconds
ARM_CHECK_TIME = 0.5
# Arming timeout in seconds
ARM_TIMEOUT = 2

'''
Class name: VehicleState
Description: VehicleState is an enumerate class which indicates the 
             current vehicle state.
'''
class VehicleState(object)
	preFlight = "PREFLIGHT"
	takeoff = "TAKEOFF"
	auto = "AUTO"
	manual = "MANUAL"
	landing = "LANDING"
	landed = "LANDED"

class Vehicle(object)
	def __init__(FCAddress = None, baudRate = 921600, self):
		# IP address for SITL simulator. Port 14550 is reserved for GCS
		# If you're running SITL, make sure mavproxy is running and Port
		# 14551 has been configured as an "--out" port
		self.SITL = "127.0.0.1:14551"
		# Connection from on-board companion computer to FC
		self.FC = FCAddress
		# Baud rate used to for serial connection
		self.BAUD = baudRate
		# Initialize the vehicle state
		self.STATE = VehicleState.preFlight
		# The vehicle object
		self.vehicle = None
		
	'''
	Function name: initialize
	Description: Connect to the vehicle with DroneKit and initialize 
	             the vehicle object
	Param: simulation - True to enable simulation, False for real vehicle connection
	Return: True - initialized successfully
	        False - initialization not successful
	'''
	def initialize(simulation = False, self):
		if self.STATE != VehicleState.preFlight:
			print "Err: Connection denied with vehicle state %s." % self.STATE
			return False
		
		if simulation：	
			connectStr = self.SITL
		else:
			connectStr = self.FC
		print "Connecting to the vehicle on %s with baud rate %d." % (connectStr, self.BAUD)
		self.vehicle = dronekit.connect(connectStr, baud = self.BAUD, wait_ready = True)
		
		if not self.vehicle:
			print "Err: Unable to connect to vehicle."
			return False
		
		print "Waiting for vehicle to initialize..."
		timeoutCounter = 0
		while not self.vehicle.is_armable:
			time.sleep(INITIALIZE_CHECK_TIME)
			if (timeoutCounter += 1) >= (INITIALIZE_TIMEOUT / INITIALIZE_CHECKTIME):
				print "Vehicle initialization timeout."
				return False
		
		print "Vehicle initialized successfully, ready for flight."
		return True
		
	'''
	Function name: switchMode
	Description: Switch to a target mode and return whether it was successful.
	Param: targetMode - the mode to be switched, "GUIDED" by default.
	Return: True - successfully switched to target mode
	        False - failed to do so
	'''
	def switchMode(targetMode = "GUIDED", self):
		self.vehicle.mode = dronekit.VehicleMode(targetMode)
		timeoutCounter = 0
		while self.vehicle.mode != dronekit.VehicleMode(targetMode):
			time.sleep(SWITCH_MODE_CHECK_TIME)
			if (timeoutCounter += 1) >= (SWITCH_MODE_TIMEOUT / SWITCH_MODE_CHECK_TIME):
				return False
		
		return True
	
	'''
	Function name: arm
	Description: Arm the vehicle and return whether it was successful.
	Param: checkGUIDED - when set True, if the vehicle were not in GUIDED mode,
	                     it would call switchMode() to switch to GUIDED mdoe.
	Return: True - armed successfully
	        False - failed to do so
	'''
	def arm(checkGUIDED = True, self):
		if checkGUIDED and self.vehicle.mode != dronekit.VehicleMode("GUIDED"):
			self.switchMode()
		
		self.vehicle.armed = True
		timeoutCounter = 0
		while not self.vehicle.armed:
			time.sleep(ARM_CHECK_TIME)
			if (timeoutCounter += 1) >= (ARM_TIMEOUT / ARM_CHECK_TIME):
				return False
		
		self.STATE = VehicleState.landed
		return True
		
	'''
	Function name: disarm
	Description: If condition permits, disarm the vehicle.
	Param: force - True to skip all checks and disarm the vehicle
	Return: True - disarmed successfully
	        False - disarm denied
	'''
	def disarm(force = False, self):
		if not force:
			if self.STATE != VehicleState.landed:
				print "Err: Cannot disarm in %s state." % self.STATE
				return False

		self.vehicle.armed = False
		return True

	'''
	Function name: takeoff
	Description: Send takeoff command for the vehicle to a target altitude.
	Param: targetHeight - target height in meters
	Return: True - takeoff command sent successfully
	        False - cannot send takeoff command
	'''
	def takeoff(targetHeight, self):
		if self.STATE != VehicleState.landed:
			print "Err: Takeoff denied with vehicle state %s." % self.STATE
			return False
		elif targetHeight <= 0:
			print "Err: Takeoff denied with invalid target height %d." % targetHeight
			return False
		else:
			self.vehicle.simple_takeoff(targetHeight)
			return True
	
	'''
	Function name: land
	Description: Switch the vehicle mode to LAND. It will land the vehicle and 
	             automatically disarm the vehicle once landed.
	Param: force - True to ignore all checks
	Return: True - landing completed
	        False - landing command was denied by vehicle
	'''
	def land(force = False, self):
		if self.STATE == VehicleState.preFlight \
		or self.STATE == VehicleState.landing \
		or self.STATE == VehicleState.landed:
			print "Err: Land denied with vehicle state %s." % self.STATE
			return False
		
		if self.switchMode("LAND"):
			self.STATE = VehicleState.landing
			print "Landing"
		else:
			print "Err: Failed to switch to LAND."
			return False
		
		while self.vehicle.armed:
			print "Landed successfully."
			self.STATE = VehicleState.landed
			return True
