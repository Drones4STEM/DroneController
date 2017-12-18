'''
File: FlightController.py
Author: Roger Wang
Description: This File contains controlling methods for drones
             implemented with Optical Flow sensor, range finder and
             a companion computer. This script is designed for indoor 
             missions.
'''

import dronekit
from pymavlink import mavutil
import time
import threading
import os
from pymavlink.dialects.v10 import ardupilotmega

# Initialization timeout in seconds
INITIALIZE_TIMEOUT = 60
# Mode switching check time period in seconds
SWITCH_MODE_CHECK_TIME = 0.5
# Mode switching timeout in seconds
SWITCH_MODE_TIMEOUT = 4
# Arming check time in seconds
ARM_CHECK_TIME = 0.5
# Arming timeout in seconds
ARM_TIMEOUT = 2
# Taking off target altitude threshold scaler
TAKEOFF_ALT_SCALER = 0.9
# Standard check time in seconds
STD_CHECK_TIME = 1
# Failsafe sleep time in seconds
FS_SLEEP_TIME = 1
# Loiter hover throttle level
LOITER_HOVER_THROTTLE = 1500

'''
Class name: VehicleState
Description: VehicleState is an enumerate class which indicates the 
             current vehicle state.
'''
class VehicleState(object):
	preFlight = "PREFLIGHT"
	takeoff = "TAKEOFF"
	auto = "AUTO"
	manual = "MANUAL"
	landing = "LANDING"
	landed = "LANDED"

'''
Class name: Custom_DroneKit_Vehicle
Description: Add some custom features to original DroneKit Vehicle class
'''
class Custom_DroneKit_Vehicle(dronekit.Vehicle):
	def __init__(self, *args):
		super(Custom_DroneKit_Vehicle, self).__init__(*args)
		
		self._ekf_predposhorizrel = False
		@self.on_message('EKF_STATUS_REPORT')
		def listener(self, name, m):
			# boolean: EKF's predicted horizontal position (relative) estimate is good
			self._ekf_predposhorizrel = (m.flags & ardupilotmega.EKF_PRED_POS_HORIZ_REL) > 0
			self.notify_attribute_listeners('is_armable', self.is_armable, cache=True)
	
	'''
	Property name: is_armable
	Description: override the is_armable property in dronekit.Vehicle, intended for
	             indoor OF + RF autonomous missions. 
	'''
	@property
	def is_armable(self):
		return self.mode != 'INITIALISING' and self.rangefinder.distance and self._ekf_predposhorizrel


'''
Class name: Vehicle
Description: Main vehicle class
'''
class Vehicle(object):
	def __init__(self, FCAddress = None, baudRate = 921600):
		# IP address for SITL simulator. Port 14550 is reserved for GCS.
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
		# The failsafe controller
		self.fsController = None

	'''
	Property name: attitude
	Description: Current vehicle attitude
	             attitude.roll - Vehicle roll in radians
				 attitude.pitch - Vehicle pitch in radians
				 attitude.yaw - Vehicle yaw in radians
	'''
	@property
	def attitude(self):
		return self.vehicle.attitude

	'''
	Property name: battery
	Description: Current battery status.
	             battery.voltage - Voltage in millivolts
				 battery.current - Current in 10*milliamperes
				 battery.level - Remaining battery energy. None if can't be estimated
	'''
	@property
	def battery(self):
		return self.vehicle.battery
		
	'''
	Property name: channels
	Description: Current RC channels input (in PWM values)
	             channels['1'] - Channel 1 reading (avaliable from 1 to 8)
	'''
	@property
	def channels(self):
		return self.vehicle.channels

	'''
	Property name: rangefinder
	Description: Current rangefinder read
	             rangefinder.distance - Range finder read in meters
	'''
	@property
	def rangefinder(self):
		return self.vehicle.rangefinder
	
	'''
	Function name: initialize
	Description: Connect to the vehicle with DroneKit and initialize the vehicle object,
	             and start the failsafe controller thread.
	Param: simulation - True to enable simulation, False for real vehicle connection
	Return: True - initialized successfully
	        False - initialization not successful
	'''
	def initialize(self, simulation = False):
		# Start the failsafe controller thread
		if not self.fsController:
			self.fsController = FailsafeController(self)
			self.fsController.start()
		
		if self.STATE != VehicleState.preFlight:
			print "Err: Connection denied with vehicle state %s." % self.STATE
			return False
		
		if simulation:
			connectStr = self.SITL
		else:
			connectStr = self.FC
		print "Connecting to the vehicle on %s with baud rate %d." % (connectStr, self.BAUD)
		self.vehicle = dronekit.connect(connectStr, baud = self.BAUD, wait_ready = True, vehicle_class = Custom_DroneKit_Vehicle)
		
		if not self.vehicle:
			print "Err: Unable to connect to vehicle."
			return False
		
		print "Waiting for vehicle to initialize..."
		timeoutCounter = 0
		# Check if the vehicle is able to arm
		while not self.vehicle.is_armable:
			time.sleep(STD_CHECK_TIME)
			timeoutCounter += 1
			#print 'vehicle mode = ', self.vehicle.mode
			#print 'ekf = ', self.vehicle._ekf_predposhorizabs
			if timeoutCounter >= (INITIALIZE_TIMEOUT / STD_CHECK_TIME):
				print "Vehicle initialization timeout."
				return False
		
		self.set_home()
		
		print "Vehicle initialized successfully, ready for flight."
		return True
		
	'''
	Function name: switch_mode
	Description: Switch to a target mode and return whether it was successful.
	Param: targetMode - the mode to be switched, "GUIDED" by default.
	Return: True - successfully switched to target mode
	        False - failed to do so
	'''
	def switch_mode(self, targetMode = "GUIDED"):
		self.vehicle.mode = dronekit.VehicleMode(targetMode)
		timeoutCounter = 0
		while self.vehicle.mode != dronekit.VehicleMode(targetMode):
			time.sleep(SWITCH_MODE_CHECK_TIME)
			timeoutCounter += 1
			if timeoutCounter >= (SWITCH_MODE_TIMEOUT / SWITCH_MODE_CHECK_TIME):
				return False
		
		return True
	
	'''
	Function name: arm
	Description: Arm the vehicle and return whether it was successful.
	Param: checkGUIDED - when set True, if the vehicle were not in GUIDED mode,
	                     it would call switch_mode() to switch to GUIDED mode.
	Return: True - armed successfully
	        False - failed to do so
	'''
	def arm(self, checkGUIDED = True):
		if checkGUIDED and self.vehicle.mode != dronekit.VehicleMode("GUIDED"):
			self.switch_mode()
		
		self.vehicle.armed = True
		timeoutCounter = 0
		while not self.vehicle.armed:
			time.sleep(ARM_CHECK_TIME)
			timeoutCounter += 1
			if timeoutCounter >= (ARM_TIMEOUT / ARM_CHECK_TIME):
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
	def disarm(self, force = False):
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
	       wait_ready - wait until the vehicle has reached the target altitude, True by default
	Return: True - takeoff command sent successfully
	        False - cannot send takeoff command
	'''
	def takeoff(self, targetHeight, wait_ready = True, relative = True):
		if self.fsController.triggered:
			return False
		if self.STATE != VehicleState.landed:
			print "Err: Takeoff denied with vehicle state %s." % self.STATE
			return False
		#elif targetHeight <= 0:
		#	print "Err: Takeoff denied with invalid target height %d." % targetHeight
		#	return False
		
		if relative:
			startHeightGlobal = self.vehicle.location.global_relative_frame.alt
			targetHeightGlobal = startHeightGlobal + targetHeight
		else:
			startHeightGlobal = 0
			targetHeightGlobal = targetHeight
		
		self.STATE = VehicleState.takeoff
		self.vehicle.simple_takeoff(targetHeightGlobal)
		print "Vehicle is taking off!"
		
		while wait_ready:
			print " Current altitude: ", self.vehicle.location.global_relative_frame.alt - startHeightGlobal 
			if self.STATE != VehicleState.takeoff:
				print "Err: Takeoff terminated unexpectedly with state %s." % self.STATE
				return False
			#Break and return from function just below target altitude.        
			if self.vehicle.location.global_relative_frame.alt - startHeightGlobal >= targetHeight * TAKEOFF_ALT_SCALER: 
				print "Reached target altitude"
				self.STATE = VehicleState.auto
				break
			time.sleep(STD_CHECK_TIME)
		return True
	
	'''
	Function name: land
	Description: Switch the vehicle mode to LAND. It will land the vehicle and 
	             automatically disarm the vehicle once landed.
	Param: force - True to ignore all checks
	Return: True - landing completed
	        False - landing command was denied by vehicle
	'''
	def land(self, force = False):
		if not force:
			if self.fsController.triggered:
				return False
			if self.STATE == VehicleState.preFlight \
			or self.STATE == VehicleState.landing \
			or self.STATE == VehicleState.landed:
				print "Err: Land denied with vehicle state %s." % self.STATE
				return False
		
		# Change the STATE first, prevent triggering failsafe incorrectly
		lastSTATE = self.STATE
		self.STATE = VehicleState.landing
		
		if self.switch_mode("LAND"):
			print "Landing"
		else:
			print "Err: Failed to switch to LAND."
			self.STATE = lastSTATE
			return False
		
		while self.vehicle.armed:
			time.sleep(STD_CHECK_TIME)
		print "Landed successfully."
		self.STATE = VehicleState.landed
		return True
		
	'''
	Function name: goto
	Description: guide the vehicle to a position relative to the vehicle
	Param: pos_x - Position forward/northward in meters
	       pos_y - Position rightward/eastward in meters
	       pos_z - Position downward in meters (should be negative)
	       relative - True for relative position(forward etc), False for absolute 
	                  position(northward etc)
	Return: True - message sent successfully
	        False - operation denied
	'''
	def goto(self, pos_x, pos_y, pos_z, relative = True):
		if self.fsController.triggered:
			return False
		if self.STATE != VehicleState.auto:
			print "Err: Goto command denied with vehicle state %s." % self.STATE
			return False
		
		# Decide which frame type to be used
		if relative:
			frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED
		else:
			frame = mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED
		
		# create the SET_POSITION_TARGET_LOCAL_NED command
		msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
						0,       # time_boot_ms (not used)
						0, 0,    # target system, target component
						frame,   # frame
						0b0000111111111000, # type_mask (only positions enabled)
						pos_x, pos_y, pos_z, # x, y, z positions
						0, 0, 0, # x, y, z velocity(not used)
						0, 0, 0, # x, y, z acceleration (not used)
						0, 0)    # yaw, yaw_rate (not used) 
		# send command to vehicle
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()
		return True

	'''
	Function name: send_nav_velocity
	Description: send_nav_velocity command to vehicle to request it fly in 
	             specified direction
	Param: velocity_x - forward/northward velocity in m/s
	       velocity_y - rightward/eastward velocity in m/s
	       velocity_z - downward velocity in m/s
	       relative - True for relative velocity(forward etc), False for absolute
	       velocity(northward etc)
	Return: True - message sent successfully
	        False - operation denied
	'''
	def send_nav_velocity(self, velocity_x, velocity_y, velocity_z, relative = True):
		if self.fsController.triggered:
			return False
		if self.STATE != VehicleState.auto:
			print "Err: Velocity control denied with vehicle state %s." % self.STATE
			return False
			
		# Decide which frame type to be used
		if relative:
			frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED
		else:
			frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
			
		# create the SET_POSITION_TARGET_LOCAL_NED command
		msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
						0,       # time_boot_ms (not used)
						0, 0,    # target system, target component
						frame,   # frame
						0b0000111111000111, # type_mask (only speeds enabled)
						0, 0, 0, # x, y, z positions (not used)
						velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
						0, 0, 0, # x, y, z acceleration (not used)
						0, 0)    # yaw, yaw_rate (not used) 
		# send command to vehicle
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()
		return True
		
	'''
	Function name: condition_yaw
	Description: send condition_yaw MAVLink command to control the vehicle's heading
	Param: heading - vehicle's target heading in degrees,
	       relative - True for relative yaw angle, False for absolute
		   clock_wise - Only meaningful when relative = True.
		                True for clock wise, False for counter clock wise
	Return: True - message sent successfully
	        False - operation denied
	'''
	def condition_yaw(self, heading, relative = True, clock_wise = True):
		if self.fsController.triggered:
			return False
		if self.STATE != VehicleState.auto:
			print "Err: Yaw control denied with vehicle state %s." % self.STATE
			return False
		
		# Yaw command in absolute or relative angle
		if relative:
			isRelative = 1
		else:
			isRelative = 0
			
		# The vehicle will rotate in cw or ccw by some degrees
		if clock_wise:
			direction = 1  # Degree described by 'heading' will be added to current degree
		else:
			direction = -1 # Degree described by 'heading' will be subscribed from current degree
		if not relative:
			direction = 0

		# create the CONDITION_YAW command
		msg = self.vehicle.message_factory.command_long_encode(
						0, 0,       # target system, target component
						mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
						0,          # confirmation
						heading,    # param 1, yaw in degrees
						0,          # param 2, yaw speed (not used)
						direction,  # param 3, direction
						isRelative, # param 4, relative or absolute degrees 
						0, 0, 0)    # param 5-7, not used
		# send command to vehicle
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()
		return True
	
	'''
	Function name: hover
	Description: command the vehicle to hold its current position. This method
	             will switch the vehicle mode to LOITER and override the throttle
	             level to LOITER_HOVER_THROTTLE
	Param: duration - time for hovering in seconds
	Return: True - message sent successfully
	        False - operation denied
	'''
	def hover(self, duration):
		if self.fsController.triggered:
			return False
		if self.STATE != VehicleState.auto:
			print "Err: Hovering denied with vehicle state %s." % self.STATE
			return False
		
		# Set RC3(throttle) to the hover level
		self.vehicle.channels.overrides['3'] = LOITER_HOVER_THROTTLE
	
		# create the LOITER_UNLIM command
		msg = self.vehicle.message_factory.command_long_encode(
						0, 0,       # target system, target component
						mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, # command
						0,          # confirmation
						0, 0, 0, 0, # param 1-4, not used
						0, 0, 0)    # param 5-7, set 0 for current position
		# send command to vehicle
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()
		
		time.sleep(duration)
		
		if not self.fsController.triggered:
			self.switch_mode()
		
		# Clear channel overrides
		self.vehicle.channels.overrides['3'] = None
		
		return True
		
	'''
	Function name: set_home
	Description: Set 'home' to a specific position. This is necessary for guiding the 
	             vehicle in GPS-denied conditions.
	             NOTE that lng and lat can't be 0, otherwise the vehicle will reject
	             this command
	Return: True - command sent successfully
	        False - operation denied
	'''
	def set_home(self, lng = 0, lat = 1, alt = 0):
		if self.STATE != VehicleState.preFlight:
			return False
		# create the SET_HOME command
		msg = self.vehicle.message_factory.command_long_encode(
						0, 0,       # target system, target component
						mavutil.mavlink.MAV_CMD_DO_SET_HOME, # command
						0,          # confirmation
						0,          # param 1, use absolute location
						0, 0, 0,    # param 2-4, not used
						lat,        # param 5, latitude
						lng,        # param 6, longitude 
						alt)        # param 7, altitude
		# send command to vehicle
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()
	
	'''
	Function name: exit
	Description: Stop all operations
	Return: True - exited successfully
	        False - Something wrong happened
	'''
	def exit(self):
		if self.STATE != VehicleState.landed and self.STATE != VehicleState.preFlight:
			print "Err: cannot exit when still in the air! State %s." % self.STATE
			return False
		
		self.fsController.join()
		self.vehicle.close()
		
		return True

'''
Class name: FailsafeController
Description: A failsafe thread handling failsafe exceptions.
'''
class FailsafeController(threading.Thread):
	def __init__(self, ctrlInstance):
		self.instance = ctrlInstance
		self.stoprequest = threading.Event()
		self.triggered = False
		super(FailsafeController, self).__init__()
	
	def run(self):
		disarmCounter = 0
		while not self.stoprequest.isSet():
			if self.instance.STATE == VehicleState.auto or self.instance.STATE == VehicleState.takeoff:
				# The vehicle is disarmed unexpectedly
				if not self.instance.vehicle.armed:
					# Counter was added against latency in 'vehicle.armed'
					disarmCounter += 1
					if disarmCounter >= 3:
						print 'Vehicle disarmed unexpectedly.'
						self.instance.STATE = VehicleState.landed
				else:
					disarmCounter = 0
				# A failsafe error will trigger the aircraft to switch into LAND or RTL mode
				if self.instance.vehicle.mode == 'LAND' or self.instance.vehicle.mode == 'RTL':
					self.triggered = True
					self.instance.vehicle.channels.overrides = None
					if self.instance.vehicle.armed:
						print 'Failsafe triggered, now landing.'
						self.instance.STATE = VehicleState.landing
					else:
						print 'Failsafe triggered, landed unexpectedly. Now exiting...'
						self.instance.STATE = VehicleState.landed
						os._exit(1)
						
				# If not in GUIDED or AUTO mode, the vehicle is controlled manually
				elif self.instance.STATE != VehicleState.manual and self.instance.vehicle.mode != 'GUIDED' \
				and self.instance.vehicle.mode != 'AUTO' and self.instance.vehicle.mode != 'LOITER':
					self.instance.STATE = VehicleState.manual
			
			if self.triggered and not self.instance.vehicle.armed and self.instance.STATE == VehicleState.landing:
				print 'Landed. Now exiting...'
				os._exit(1)

			time.sleep(FS_SLEEP_TIME)
		
	def join(self, timeout=None):
		self.stoprequest.set()
		super(FailsafeController, self).join(timeout)
