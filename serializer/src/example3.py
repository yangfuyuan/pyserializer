"""
    Example 3: Multiple sensors.
    
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
import time, os
    
if os.name == "posix":
    portName = "/dev/ttyUSB0" # Change this to your main Serializer port!
    # portName = "/dev/rfcomm0" # For bluetooth on Linux
    # Note: On Linux, after connecting to the Bluetooth adapter, run the command
    # sudo rfcomm bind /dev/rfcomm0
else:
    portName = "COM12" # Change this to your main Serializer port!
    
baudRate = 19200 # Change this to your Serializer baud rate!

mySerializer = Serializer.Serializer(port=portName, baudrate=baudRate, timeout=0.5)
mySerializer.connect()

""" The following sensors were attached to the corresponding pins for this example.
    * Ping Sonar on GPIO pin 4
    * Phidgets Temperature Sensor on Analog pin 0
    * Phidgets 20 amp Current Sensor on Analog pin 1
    * Sharp GP2D12 IR sensor on Analog pin 4
    * SFE Laser pointer module on GPIO pin 1
    * HiTec servo on GPIO pin 5
    * Devantech SP03 speech module on the I2C bus
    * Devantech CMPS03 compass module on the I2C bus
"""

myPing = Serializer.Ping(mySerializer, 4)  
myIR = Serializer.GP2D12(mySerializer, 4)
myTemp = Serializer.PhidgetsTemperature(mySerializer, 0, "F")
myAmps = Serializer.PhidgetsCurrent(mySerializer, 1, model=20, ac_dc="dc")

print "Firmware Version", mySerializer.fw()
print "Units", mySerializer.get_units()
print "Baudrate", mySerializer.get_baud()
print "Encoder type", mySerializer.get_encoder()
print "DPID params", mySerializer.get_dpid()
print "VPID params", mySerializer.get_vpid()
print "Raw analog port values for a few pins", mySerializer.sensor([0, 3, 5])
print "All Analog Sensor Values:", mySerializer.get_all_analog()
print "Analog values from the cache:", mySerializer.analog_sensor_cache
print "Serializer voltage from the cache", mySerializer.voltage(cached=True)
print "Temperature in Fahrenheit:", round(myTemp.value(), 1)
print "Current in Amps:", round(myAmps.value(), 2)
print "Ping Sonar reading on digital pin 4:", mySerializer.pping(4)
print "Ping reading using the Ping class and reading from the cache:", myPing.value(cached=True)
print "Sharp IR reading on analog pin 4:", myIR.value()
print "Sharp IR reading from cache:", myIR.value(cached=True)
print "Setting GPIO pin 1 to off turns the SFE laser on", mySerializer.set_io(1, 0)
print "Getting values on GPIO pin 8 and 9:", mySerializer.get_io([8, 9])
#print "SP03 Speech Chip on I2C bus", mySerializer.sp03("Greetings Humans!  Welcome to the future.")
#print "Say 'Hello' through the SP03 Speech Chip using the raw I2C command", mySerializer.i2c("w", 196, "0 0 0 5 3 72 101 108 108 111 0")
#print "Read back a given number of bytes from the I2C device:", mySerializer.i2c("r", 196, "2")
#print "Execute the pre-made speech command", mySerializer.i2c("w", 196, "0 64")
#print "Devantech Compass on I2C bus:", mySerializer.get_compass()
#print "Devantech Compass reading using raw I2C commands", mySerializer.i2c("w", 192, "1"), mySerializer.i2c("r", 192, "1")
print "Blinking the LEDs for 3 seconds"
mySerializer.blink_led([1,2], [100, 100])
time.sleep(3)
mySerializer.blink_led([1,2], [0, 0])
print "Test servo on GPIO pin 5 (servo ID 6)", mySerializer.servo(6, 0)

""" Poll a number of sensors 50 times at a rate of 10 times per second. """
for i in range(50):
    start = time.time()
    sonar = myPing.value()
    mySerializer.get_all_analog()
    ir = myIR.value(cached=True)
    volts = mySerializer.voltage(cached=True)
    amps = myAmps.value(cached=True)
    deltaT = time.time() - start
    time.sleep(max(0, (0.1 - deltaT))) # 10Hz
    print "Time:", round(time.time() - start, 3), "Sonar:", sonar, "IR:", round(ir, 1), "Volts:", round(volts, 2), "Amps:", round(amps, 2)

print "\nTesting completed, shutting down."

mySerializer.stop()
mySerializer.close()

