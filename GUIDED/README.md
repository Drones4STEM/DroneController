# DroneController/GUIDED
Libraries and examples used to control the drone's movements in GUIDED mode, if the drone is implemented with an **optical flow sensor** and a **rangefinder**. 

Velocity control, position control and yaw control are currently avaliable by``import FlightController`` , and some example scripts are provided to control the vehicle with SITL, or a real drone. If you have a good understanding of ArduCopter, SITL and Companion Computers, you can get it worked with these provided examples quickly.

See the following video of a real quad copter test. The script is avaliable in **GUIDED/Velocity&YawCtrl.py**

[![Video_frame](https://img.youtube.com/vi/HeJs-0_Bv8s/0.jpg)](https://www.youtube.com/watch?v=HeJs-0_Bv8s)

### File Description

**FlightController.py** - The library used to control an ArduCopter vehicle with optical flow sensor and rangefinder. (Things are little bit different from controlling a drone outdoors with GPS)

**TakeoffAndLand.py** - Example which commands the vehicle to initialize, arm, takeoff for 1m and then disarm.

**Velocity&YawCtrl.py** - Example which commands the vehicle to change its velocity, yaw(heading) and position indoors. [Real-vehicle test video provided]( https://youtu.be/HeJs-0_Bv8s). 

**VehicleProperty.py** - Example which shows how to read some useful attributes from the vehicle. 
