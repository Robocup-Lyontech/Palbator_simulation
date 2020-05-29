#!/usr/bin/env python
import tf
import math
import numpy as np
import rospy
from tf import TransformListener
from geometry_msgs.msg import PointStamped

class PointTo:

    def __init__(self):

        rospy.init_node("test_changement_repere")
        
        listener = TransformListener()
        now = rospy.Time(0)
        object_point = PointStamped()
        object_point.header.frame_id = "palbator_arm_kinect_link"
        object_point.header.stamp = now
        object_point.point.x = 2.15
        object_point.point.y = 0.44
        object_point.point.z = 0.0

        rospy.loginfo("{class_name} : Object coords in palbator_arm_kinect_link : %s".format(class_name=self.__class__.__name__),str(object_point))
        listener.waitForTransform("map", "/palbator_arm_kinect_link", now, rospy.Duration(20))
        target = listener.transformPoint("/map",object_point)

        rospy.loginfo("{class_name} : Object coords in map : %s".format(class_name=self.__class__.__name__),str(target))




if __name__ == "__main__":

    pointing_instance = PointTo()

