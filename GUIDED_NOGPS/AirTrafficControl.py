############################################
# This file contains a wrapper class
# for DroneKit related operations
# for our drone.
############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science Technology
# Spring 2017
# Lucas Coon, Mark Raymond Jr.
# pylint: disable=C, F, I, R, W

from datetime import datetime, timedelta
from os import system
import sys
from time import sleep
from copy import deepcopy
# from Collision import Sonar

# import RPi.GPIO as GPIO
import dronekit
import math
import os
import time
import threading

class DroneAttitude():

  def __init__(self, roll, pitch, yaw):
    self.pitch_deg = pitch
    self.yaw_deg = yaw
    self.roll_deg = roll
    self.pitch = math.radians(pitch)
    self.yaw = math.radians(yaw)
    self.roll = math.radians(roll)
    self.quaternion = self.get_quaternion()

  def get_quaternion(self):
    q = []

    t0 = math.cos(self.yaw * 0.5)
    t1 = math.sin(self.yaw * 0.5)
    t2 = math.cos(self.roll * 0.5)
    t3 = math.sin(self.roll * 0.5)
    t4 = math.cos(self.pitch * 0.5)
    t5 = math.sin(self.pitch * 0.5)

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    q.append(w)
    q.append(x)
    q.append(y)
    q.append(z)

    return q

class StandardAttitudes(object):
  level = DroneAttitude(0,0,0)
  forward = DroneAttitude(0,-5,0)
  backward = DroneAttitude(0,5,0)
  left = DroneAttitude(-5, 0, 0)
  right = DroneAttitude(5, 0, 0)

class StandardThrusts(object):
  none = 0.00
  hover = 0.525
  takeoff = 0.75
  full = 1.00

class VehicleStates(object):
  hover = "HOVER"
  flying = "FLYING"
  takeoff = "TAKEOFF"
  unknown = "UNKNOWN"
  avoidance = "AVOIDANCE"
  landing = "LANDING"
  landed = "LANDED"

class Tower(object):
  SIMULATOR = "127.0.0.1:14551"
  USB = "/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00"
  USB_DEV = "/dev/cu.usbmodem1"
  STANDARD_ATTITUDE_BIT_FLAGS = 0b00111111
  FLIP_ATTITUDE_BIT_FLAGS = 0b00111000
  STANDARD_THRUST_CHANGE = 0.05
  MAX_TURN_TIME = 5
  LAND_ALTITUDE = 0.5
  TURN_START_VELOCITY = 3
  TURN_RADIUS = 0.5 # Meters
  STANDARD_ANGLE_ADJUSTMENT = 1.0
  MESSAGE_WAIT_TIME = 0.01
  ACCEL_NOISE_THRESHOLD = 0.05
  MAX_ANGLE_ALL_AXIS = 15.0
  BATTERY_FAILSAFE_VOLTAGE = 9.25
  FAILSAFES_SLEEP_TIME = 0.1
  STANDARD_SLEEP_TIME = 1
  BAUDRATE = 115200

  def __init__(self):
    self.start_time = 0
    self.flight_log = None
    self.vehicle_initialized = False
    self.vehicle = None
    self.LAST_ATTITUDE = StandardAttitudes.level
    self.LAST_THRUST = StandardThrusts.none
    self.STATE = VehicleStates.unknown

  def initialize(self, should_write_to_file=False):
    """ 
    @purpose: Connect to the flight controller, start the failsafe
              thread, switch to GUIDED_NOGPS, and open a file to 
              begin logging.
    @args:
    @returns:
    """
    if(not self.vehicle_initialized):

      if(should_write_to_file):
        self.flight_log = open('flight_log.txt', 'w')
        sys.stdout = self.flight_log

      print("\nConnecting via USB to PixHawk...")
	  #Add baud_rate parameter
      self.vehicle = dronekit.connect(self.USB, baud = self.BAUDRATE, wait_ready=True)

      if not self.vehicle:
        print("\nUnable to connect to vehicle.")
        return

      self.vehicle.mode = dronekit.VehicleMode("STABILIZE")
      self.STATE = VehicleStates.landed
      self.vehicle_initialized = True
      self.failsafes = FailsafeController(self)
      self.failsafes.start()
      self.start_time = time.time()
      
      self.switch_control()
      
      print("\nSuccessfully connected to vehicle.")

  def shutdown(self):    
    """ 
    @purpose: Stop all operations and cleanup the vehicle object.
    @args:
    @returns:
    """
    self.failsafes.join()
    self.vehicle.close()
    self.flight_log.close()
    self.vehicle_initialized = False
    self.start_time = 0

  def arm_drone(self):
    """ 
    @purpose: Arm the vehicle.
    @args:
    @returns:
    """
    self.vehicle.armed = True
    while(not self.vehicle.armed):
      sleep(self.STANDARD_SLEEP_TIME)

  def disarm_drone(self):
    """ 
    @purpose: Disarm the vehicle.
    @args:
    @returns:
    """
    self.vehicle.armed = False
    while(self.vehicle.armed):
      sleep(self.STANDARD_SLEEP_TIME)

  def switch_control(self):
    """ 
    @purpose: Switch the mode to GUIDED_NOGPS and make sure 
             that the failsafe thread is running.
    @args:
    @returns:
    """
    if not self.failsafes:
      self.failsafes = FailsafeController(self)
      self.failsafes.start()
    if self.vehicle.mode.name != "GUIDED_NOGPS":
      self.vehicle.mode = dronekit.VehicleMode("GUIDED_NOGPS")
      while(self.vehicle.mode.name != "GUIDED_NOGPS"):
        sleep(self.STANDARD_SLEEP_TIME)

  def get_uptime(self):
    """ 
    @purpose: Get up time of this object.
    @args:
    @returns:
    """
    uptime = time.time() - self.start_time
    return uptime

  def map(self, x, in_min, in_max, out_min, out_max):
    """ 
    @purpose: Re-maps a number from one range to another.
    
    @args: 
      x: the number to map
      in_min: the lower bound of the value's current range
      in_max: the upper bound of the value's current range
      out_min: the lower bound of the value's target range
      out_max: the upper bound of the value's target range

    @returns:
      The mapped value.
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

  def set_angle_thrust(self, attitude, thrust):
    """ 
    @purpose: Send an specified attitude message to the
              flight controller. For more information, see
              http://mavlink.org/messages/common#SET_ATTITUDE_TARGET.
    
    @args: 
      attitude: A DroneAtittude object containing a target attitude.
      thrust: A collective thrust from 0 to 1. Thrust is converted to
              a climb rate internally by the flight controller. Therefore, 
              thrusts from 0.51 to 1 are climb rates and thrusts from 0.49
              to 0 are descent rates. 0.50 attempts to maintain a hover.
    @returns:
    """
    while(self.vehicle.mode.name != "GUIDED_NOGPS"):
      sleep(self.STANDARD_SLEEP_TIME)
    
    message = self.vehicle.message_factory.set_attitude_target_encode(
      0,                                 # Timestamp in milliseconds since system boot (not used).
      0,                                 # System ID
      0,                                 # Component ID
      self.STANDARD_ATTITUDE_BIT_FLAGS,  # Bit flags. 
      attitude.quaternion,               # Attitude quaternion.
      0,                                 # Body roll rate.
      0,                                 # Body pitch rate.
      0,                                 # Body yaw rate.
      thrust                             # Collective thrust, from 0-1.
    )
    self.vehicle.send_mavlink(message)
    self.vehicle.commands.upload()
    self.last_attitude = attitude
    self.last_thrust = thrust
  
  def hover(self, duration=None):
    self.set_angle_thrust(StandardAttitudes.level, StandardThrusts.hover)

  def takeoff(self, target_altitude):

    self.STATE = VehicleStates.takeoff

    self.arm_drone()
    self.switch_control()

    initial_alt = self.vehicle.location.global_relative_frame.alt

    while((self.vehicle.location.global_relative_frame.alt - initial_alt) < target_altitude):
      self.set_angle_thrust(StandardAttitudes.level, StandardThrusts.takeoff)
      sleep(self.STANDARD_SLEEP_TIME)

    print('Reached target altitude:{0:.2f}m'.format(self.vehicle.location.global_relative_frame.alt))

#    self.hover(10)
#    self.land()

  def fly_for_time(self, duration, direction, target_velocity, should_hover_on_finish):
    end_manuever = datetime.now() + timedelta(seconds=duration)

    self.STATE = VehicleStates.flying
    self.set_angle_thrust(direction, StandardThrusts.hover)

    while(datetime.now() < end_manuever):

      print datetime.now()
      print end_manuever
      print 'Airspeed = %f , ' % self.vehicle.airspeed,
      print 'Velocity = ',
      print (self.vehicle.velocity),

      updated_attitude = deepcopy(self.LAST_ATTITUDE)

      if(self.vehicle.airspeed < target_velocity):
        updated_attitude.pitch_deg -= 1
      elif(self.vehicle.airspeed > target_velocity):
        updated_attitude.pitch_deg += 1
      else:
        updated_attitude.pitch_deg = direction.pitch_deg

      if(updated_attitude.pitch_deg < -self.MAX_ANGLE_ALL_AXIS):
        updated_attitude.pitch_deg = -self.MAX_ANGLE_ALL_AXIS

      if(updated_attitude.pitch_deg > self.MAX_ANGLE_ALL_AXIS):
        updated_attitude.pitch_deg = self.MAX_ANGLE_ALL_AXIS

      if(updated_attitude.roll_deg < -self.MAX_ANGLE_ALL_AXIS):
        updated_attitude.roll_deg = -self.MAX_ANGLE_ALL_AXIS
      
      if(updated_attitude.roll_deg > self.MAX_ANGLE_ALL_AXIS):
        updated_attitude.roll_deg = self.MAX_ANGLE_ALL_AXIS

      if(updated_attitude.yaw_deg < -self.MAX_ANGLE_ALL_AXIS):
        updated_attitude.yaw_deg = -self.MAX_ANGLE_ALL_AXIS

      if(updated_attitude.yaw_deg > self.MAX_ANGLE_ALL_AXIS):
        updated_attitude.yaw_deg = self.MAX_ANGLE_ALL_AXIS

      updated_attitude.pitch = math.radians(updated_attitude.pitch_deg)
      updated_attitude.quaternion = updated_attitude.get_quaternion()

      self.set_angle_thrust(updated_attitude, self.LAST_THRUST)

      print 'Updated Pitch = %d , ' % updated_attitude.pitch_deg

      sleep(self.STANDARD_SLEEP_TIME)
    
    if(should_hover_on_finish):
      # self.hover()
      pass

  def land(self):
    self.vehicle.mode = dronekit.VehicleMode("LAND")
    self.STATE = VehicleStates.landing
    while((self.vehicle.location.global_relative_frame.alt) >= self.LAND_ALTITUDE):
      sleep(self.STANDARD_SLEEP_TIME)
    else:
      self.STATE = VehicleStates.landed

  def do_circle_turn(self, desired_angle, direction, duration):
    if(duration > self.MAX_TURN_TIME):
      return

    self.STATE = VehicleStates.flying
      
    max_angle = math.radians(desired_angle)
    altitude_to_hold = self.vehicle.location.global_relative_frame.alt

    duration = timedelta(seconds=duration)
    end_manuever = datetime.now() + duration
    
    self.fly_for_time(1, StandardAttitudes.forward, self.TURN_START_VELOCITY, False)
    
    while(end_manuever <= datetime.now()):
      change_in_time = end_manuever - datetime.now()
      current_altitude = self.vehicle.location.global_relative_frame.alt

      roll_angle = max_angle * (math.cos(self.vehicle.airspeed * change_in_time.seconds) / self.TURN_RADIUS)
      pitch_angle = max_angle * (math.sin(self.vehicle.airspeed * change_in_time.seconds) / self.TURN_RADIUS)

      roll_angle = math.degrees(roll_angle)
      pitch_angle = math.degrees(pitch_angle)
      self.last_attitude.yaw = math.degrees(self.last_attitude.yaw)

      updated_attitude = DroneAttitude(pitch_angle, self.last_attitude.yaw, roll_angle)

      self.set_angle_thrust(updated_attitude, StandardThrusts.hover)

      print("Sent message.")

      if(current_altitude > altitude_to_hold):
        max_angle = math.radians(desired_angle + self.STANDARD_ANGLE_ADJUSTMENT)
      elif(current_altitude < altitude_to_hold):
        max_angle = math.radians(desired_angle - self.STANDARD_ANGLE_ADJUSTMENT)
      else:
        max_angle = math.radians(desired_angle)

      sleep(self.STANDARD_SLEEP_TIME)
      
    self.fly_for_time(1, StandardAttitudes.forward, self.vehicle.airspeed, True)
    
  def check_sonar_sensors(self):
    # sonar = Sonar.Sonar(2,3, "Main")
    # print("%s Measured Distance = %.1f cm" % (sonar.getName(), sonar.getDistance()))
    # message = vehicle.message.distance_sensor(
    #     0,
    #     6,
    #     200,
    #     sonar.getDistance(),
    #     MAV_DISTANCE_SENSOR_ULTRASOUND,
    #     158,
    #     180,
    #     0
    # )
    pass

  def check_battery_voltage(self):
    if(self.vehicle.battery.voltage < self.BATTERY_FAILSAFE_VOLTAGE):
        self.land()

class FailsafeController(threading.Thread):

  def __init__(self, atc_instance):
    self.atc = atc_instance
    self.stoprequest = threading.Event()
    super(FailsafeController, self).__init__()

  def run(self):
    while not self.stoprequest.isSet():
      if self.atc.STATE == VehicleStates.hover or self.atc.STATE == VehicleStates.flying:
        #self.atc.check_sonar_sensors()
        #self.atc.check_battery_voltage()
        sleep(self.atc.FAILSAFES_SLEEP_TIME)

  def join(self, timeout=None):
    if self.atc.vehicle.armed:
      if self.atc.STATE != VehicleStates.landed:
        self.atc.land()

    self.stoprequest.set()
    # GPIO.cleanup()
    super(FailsafeController, self).join(timeout)

