#!/usr/bin/env python

import rospy

from pmb2_apps.msg import ArmControlGoal, ArmControlAction
import actionlib

class ArmControl:

    def __init__(self):

        rospy.init_node("arm_control_API",anonymous=True)

        self.client = actionlib.SimpleActionClient('Moveit_Palbator_global_action', ArmControlAction)

        rospy.loginfo("WAITING FOR SERVER")
        self.client.wait_for_server()

        rospy.loginfo("ARM CONTROL CLIENT INIT")

        rospy.sleep(3)

        self.goal = ArmControlGoal()
        self.goal.action = 'PointingXYZ'
        # self.goal.object_label = 'object_bleach0_TF'
        self.goal.coord_x = 1.5
        self.goal.coord_y = 5.5
        self.goal.coord_z = 1.2

        self.client.send_goal(self.goal)

        self.client.wait_for_result()

        result_action = self.client.get_result()

        rospy.loginfo("action result : %s",result_action)

if __name__ == "__main__":
    client_armcontrol = ArmControl()
    while not rospy.is_shutdown():
        rospy.spin()


