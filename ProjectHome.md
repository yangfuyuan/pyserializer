This is a simple Python library written for the Serializer microcontroller made by Robotics Connection. (As of 2012, the Serializer is now called the Element and is sold by cmRobot.com.) The library uses PySerial to access the firmware commands of the Serializer as well as provides some convenience functions for specific sensors such as the Phidgets temperature, voltage and current sensors and the Sharp GP2D12 IR sensor.  It also includes two PID drive commands: rotate(angle, speed) and travel\_distance(distance, speed) for controlling the drive motors.

WINDOWS USERS PLEASE NOTE: pySerializer will not run under PySerial 2.5 due to a bug in readline() in PySerial 2.5.  Please use PySerial 2.4 + pywin instead.

Most of the Serializer functions have been implemented though a few have not been tested since I don't currently have some of the supported sensors.  The functions that have **NOT** been tested are:

  * step (used with a bipolar stepper motor)
  * sweep (also used with a bipolar stepper motor)
  * srf04, srf08 and srf10 (used with a Devantech SRF04, SRF08 and SRF10 sonar sensors)
  * tpa81 (used with the Devantech TPA81 thermopile sensor)

Requirements:
> LINUX
  * Python 2.6.5 or higher
  * PySerial 2.3 or higher

> WINDOWS
  * Python 2.6.5 or higher
  * PySerial 2.4 + pywin (will not run with PySerial 2.5)

Tested on Windows XP and Ubuntu Linux 9.10 and 10.04

Documentation for this Python Serializer library can be found at:

http://www.pirobot.org/code/serializer.html


NOTE: The official SerializerTM manual can be found at:
> http://www.roboticsconnection.com/multimedia/docs/Serializer_3.0_UserGuide.pdf
