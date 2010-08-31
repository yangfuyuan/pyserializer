"""
    Example 4: Accessing the Serializer using multiple threads.
    The Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All right reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    See http://www.gnu.org/licenses/gpl.html for details.
      
    NOTE: See the offical SerializerTM manual at:
    http://www.roboticsconnection.com/multimedia/docs/Serializer_3.0_UserGuide.pdf
"""

import serializer as Serializer
import threading, time, os

if os.name == "posix":
    portName = "/dev/ttyUSB0" # Change this to your main Serializer port!
    # portName = "/dev/rfcomm0" # For bluetooth on Linux
    # Note: On Linux, after connecting to the Bluetooth adapter, run the command
    # sudo rfcomm bind /dev/rfcomm0
else:
    portName = "COM12" # Change this to your main Serializer port!
    
baudRate = 19200 # Change this to your Serializer baud rate!

mySerializer = Serializer(port=portName, baudrate=baudRate, timeout=1)

""" The following two lines assume we have a Ping sonar sensor attached to
    GPIO pin 4 and a Sharp GP2D12 IR sensor to anlog pin 4.
"""

myPing = Serializer.Ping(mySerializer, 4)
myIR = Serializer.GP2D12(mySerializer, 4)

try:
    print "Connecting to Serializer on port", portName, "...",
    mySerializer.connect()
    print "Connected!"
except:
    print "Cannot connect to Serializer!"
    os._exit(1)

class Thread1(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.finished = threading.Event()
        self.interval = 0.05 # Access the sensors 20 times per second.
        self.daemon = False
        self.count = 0

    def run(self):
        while not self.finished.isSet():
            print "Reading from Thread 1 IR:", round(myIR.value(), 1), "Sonar:", myPing.value()
            time.sleep(self.interval)
            
    def stop(self):
        print "Stopping Node 1 Thread ...",
        self.finished.set()
        self.join()
        print "Done."
            
class Thread2(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.finished = threading.Event()
        self.interval = 0.5  # Access the voltage 2 times per second.
        self.daemon = False
        self.count = 0

    def run(self):
        while not self.finished.isSet():
            print "Reading from Thread 2 Serializer Voltage", round(mySerializer.voltage(), 1)
            time.sleep(self.interval)
            
    def stop(self):
        print "Stopping Node 2 Thread ...",
        self.finished.set()
        self.join()
        print "Done."

thread1 = Thread1()
thread2 = Thread2()

thread1.start()
thread2.start()

time.sleep(5)

thread1.stop()
thread2.stop()

mySerializer.stop()
mySerializer.close()

