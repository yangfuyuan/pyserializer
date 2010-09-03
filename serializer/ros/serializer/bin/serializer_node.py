#!/usr/bin/env python
import os
import sys
sys.path.append("/home/patrick/ros/ros/core/roslib/src")
os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'
#os.environ['ROS_ROOT'] = '/home/patrick/ros/ros'
import roslib; roslib.load_manifest('serializer')
import rospy
from std_msgs.msg import String
import serializer as SerializerAPI

def SerializerROS():
    pub = rospy.Publisher('sensors', String)
    rospy.init_node('serializer')
    mySerializer = SerializerAPI.Serializer()
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        SerializerROS()
    except rospy.ROSInterruptException: pass
