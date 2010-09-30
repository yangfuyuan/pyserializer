"""
    Example 2: Use PID control to move the robot.
    
    The Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All right reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
    NOTE: See the offical SerializerTM manual at:
    http://www.roboticsconnection.com/multimedia/docs/Serializer_3.0_UserGuide.pdf
"""

import serializer as Serializer
import time
import os
import math
    
if os.name == "posix":
    portName = "/dev/ttyUSB0" # Change this to your main Serializer port!
    #portName = "/dev/rfcomm0" # For bluetooth on Linux
    # Note: On Linux, after connecting to the Bluetooth adapter, run the command
    # sudo rfcomm bind /dev/rfcomm0
else:
    portName = "COM12" # Change this to your main Serializer port!
    
baudRate = 19200 # Change this to your Serializer baud rate!

mySerializer = Serializer.Serializer(port=portName, baudrate=baudRate, timeout=0.5)
mySerializer.connect()

print "Firmware Version", mySerializer.fw()
print "Units", mySerializer.get_units()

""" * * * * *
    Caution!  Uncomment the following functions ONLY when you know your
    robot is ready to move safely!  You should also set the vpid, dpid, wheel diameter,
    wheel track and gear_reduction to match your robot.  (Default units are meters and radians.)
"""

#mySerializer.set_units(0) # Set units to metric.
#mySerializer.set_vpid(2, 0, 5, 45)
#mySerializer.set_dpid(1, 0, 0, 5, 10)
#mySerializer.set_wheel_diameter(0.132)  # Wheel diameter in meters
#mySerializer.set_wheel_track(0.3365)    # Wheel track in meters
#mySerializer.set_gear_reduction(1.667)  # 60/36 tooth exernal gear ratio
#
#mySerializer.travel_distance(0.5, 0.2) # Travel 0.5 meters forward at 0.2 meters per second.
#while mySerializer.get_pids():         # Test for completion of the movement.
#    time.sleep(0.1)
#    print "Wheel velocities", mySerializer.vel_m_per_s(), "Encoder counts:", mySerializer.get_encoder_count([1, 2])
#
#
#mySerializer.rotate(math.radians(90), 0.5) # Rotate 90 degrees at speed 0.5 radians per second.
#while mySerializer.get_pids():
#    time.sleep(0.1)
#    print "Wheel velocities", mySerializer.vel_m_per_s(), "Encoder counts:", mySerializer.get_encoder_count([1, 2])

print "\nTesting completed, shutting down."

mySerializer.stop()
mySerializer.close()

