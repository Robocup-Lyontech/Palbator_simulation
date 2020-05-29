#!/usr/bin/env python

import rospy
import actionlib

from pmb2_apps.msg import SdfInGazeboAction, SdfInGazeboGoal

class Test():

    def __init__(self):
        rospy.init_node("test_client")

        self.client = actionlib.SimpleActionClient("sdf_in_gazebo_action",SdfInGazeboAction)
        self.client.wait_for_server()
        rospy.loginfo("client connected")

        goal = SdfInGazeboGoal()

        goal.model_name = "mustard_test"
        goal.path_to_sdf = "/home/student/Bureau/global_palbator/src/Palbator_simulation/pmb2_simulation/pmb2_gazebo/models/ycb-mustard/ycb-mustard.sdf"
        

        self.client.send_goal(goal)
        self.client.wait_for_result()
        response = self.client.get_result()

        rospy.loginfo("RESPONSE FROM GAZEBO : %s",str(response))

if __name__ == "__main__":
    a=Test()