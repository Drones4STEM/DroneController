# DroneController
Scripts to control the ArduCopter drone indoors via MAVLink protocol.

## GUIDED

Libraries and examples used to control the drone's movements in GUIDED mode, if the drone is implemented with an optical flow sensor and a rangefinder.

Velocity control, position control and yaw control are currently avaliable by``import FlightController`` , and an example script was provided to control the vehicle with SITL, or a real drone.

See https://youtu.be/HeJs-0_Bv8s for the video of a real quad copter test. The script is avaliable in **GUIDED/Velocity&YawCtrl.py**

## GUIDED_NOGPS

Take off, and control the drone's movement in GUIDED_NOGPS mode.

**Special Thanks** to [Missouri S&T Multi-Rotor Robot Design Team](https://github.com/MST-MRR) and [Mark Raymond Jr](https://github.com/markrjr). Controlling methods in GUIDED_NOGPS are based on their valuable work.

