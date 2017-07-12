from AirTrafficControl import *

#Create instance
MyCopter = Tower()
#Uncomment the next two lines for serial address
#MyCopter.USB = "/dev/serial0"
#MyCopter.BAUDRATE = 921600
#Uncomment the next two lines for sitl
MyCopter.USB = "127.0.0.1:14551"
MyCopter.BAUDRATE = 115200
#Establish connection, start failsafe and switch to GUIDED_NOGPS
MyCopter.initialize()

tmp_inp = ''
print 'Copter initialized. Enter to arm the drone.'

tmp_inp = raw_input()

MyCopter.takeoff(15)

MyCopter.hover()

#MyCopter.land()
while(1):
    pass