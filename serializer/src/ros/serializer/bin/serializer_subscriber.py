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
os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'#os.environ['ROS_ROOT'] = '/home/patrick/ros/ros'

import roslib; roslib.load_manifest('serializer')
import rospy
from std_msgs.msg import Float32
#import driver.serializer as SerializerAPI
import serializer_driver as SerializerAPI 

def SerializerROS():
    pub = rospy.Publisher('sensors', Float32)
    rospy.init_node('serializer')
    rate = rospy.Rate(10)
    mySerializer = SerializerAPI.Serializer(port="/dev/ttyUSB0")
    mySerializer.connect()
    sensors = dict({})
    head_sonar = SerializerAPI.Ping(mySerializer, 4)
    head_ir = SerializerAPI.GP2D12(mySerializer, 4)
    while not rospy.is_shutdown():
        #str = "hello world %s"%rospy.get_time()
        sensors['head_ir'] = head_sonar.value()
        sensors['head_sonar'] = head_ir.value()
        rospy.loginfo(sensors)
        pub.publish(Float32(sensors))
        rate.sleep()
if __name__ == '__main__':
    try:
        SerializerROS()
    except rospy.ROSInterruptException: pass
