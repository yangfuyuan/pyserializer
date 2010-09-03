#!/usr/bin/env python
"""
    A ROS node wrapper for the PySerialzier library for the Robotics Connection 
    SerializerTM micro controller.
    
    Created for The Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.
"""
import os
import sys
sys.path.append("/home/patrick/ros/ros/core/roslib/src")
os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'
#os.environ['ROS_ROOT'] = '/home/patrick/ros/ros'
import roslib; roslib.load_manifest('serializer')
import rospy
from std_msgs.msg import Int32
import driver.serializer as SerializerAPI

def SerializerROS():
    pub = rospy.Publisher('sonar', Int32)
    rospy.init_node('serializer')
    mySerializer = SerializerAPI.Serializer(port="/dev/ttyUSB0")
    mySerializer.connect()
    while not rospy.is_shutdown():
        #str = "hello world %s"%rospy.get_time()
        sonar = mySerializer.pping(4)
        rospy.loginfo(sonar)
        pub.publish(Int32(sonar))
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        SerializerROS()
    except rospy.ROSInterruptException: pass
