#!/usr/bin/env python
import os
os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'
import roslib; roslib.load_manifest('serializer')
import rospy
from std_msgs.msg import String
import serializer as Serializer

def SerializerROS():
    pub = rospy.Publisher('sensors', String)
    rospy.init_node('serializer')
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        SerializerROS()
    except rospy.ROSInterruptException: pass
