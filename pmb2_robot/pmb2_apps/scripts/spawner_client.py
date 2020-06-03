#!/usr/bin/env python

import rospy
import actionlib

from pmb2_apps.msg import SdfInGazeboAction, SdfInGazeboGoal
from geometry_msgs.msg import Pose
import tf



class Test():

    def __init__(self):
        rospy.init_node("test_client")

        self.client = actionlib.SimpleActionClient("sdf_in_gazebo_action",SdfInGazeboAction)
        self.client.wait_for_server()
        rospy.loginfo("client connected")

        rospy.sleep(3)

        self.spawner_demo()


    def spawner_demo(self):
        rospy.loginfo("G1_entrance")
        self.spawner_receptionist("G1_entrance")
        rospy.sleep(3)
        rospy.loginfo("G1_before_present")
        self.spawner_receptionist("G1_before_present")
        rospy.sleep(3)
        rospy.loginfo("G1_after_present")
        self.spawner_receptionist("G1_after_present")
        rospy.sleep(3)
        rospy.loginfo("G2_entrance")
        self.spawner_receptionist("G2_entrance")
        rospy.sleep(3)
        rospy.loginfo("G2_before_present")
        self.spawner_receptionist("G2_before_present")
        rospy.sleep(3)

    def spawner_receptionist(self,mode):

        if mode == "G1_entrance":
            goal = SdfInGazeboGoal()
            goal.action = "spawn"
            goal.model_name = "Guest_1"
            goal.path_to_sdf = "/home/student/Bureau/global_palbator/src/Palbator_simulation/pmb2_simulation/pmb2_gazebo/models/citizen_extras_female_03/model.sdf"
            goal.model_pose = Pose()
            goal.model_pose.position.x = -0.2
            goal.model_pose.position.y = -5.5
            q = tf.transformations.quaternion_from_euler(0.011275, 0.012812, -3.088)
            goal.model_pose.orientation.x = q[0]
            goal.model_pose.orientation.y = q[1]
            goal.model_pose.orientation.z = q[2]
            goal.model_pose.orientation.w = q[3]

            self.client.send_goal(goal)
            self.client.wait_for_result()
            response = self.client.get_result()
            rospy.loginfo("RESPONSE FROM GAZEBO : %s",str(response))

        elif mode == "G1_before_present":
            goal = SdfInGazeboGoal()
            goal.action = "delete"
            goal.model_name = "Guest_1"
            self.client.send_goal(goal)
            self.client.wait_for_result()
            response = self.client.get_result()
            rospy.loginfo("RESPONSE FROM GAZEBO : %s",str(response))

            goal = SdfInGazeboGoal()
            goal.action = "spawn"
            goal.model_name = "Guest_1"
            goal.path_to_sdf = "/home/student/Bureau/global_palbator/src/Palbator_simulation/pmb2_simulation/pmb2_gazebo/models/citizen_extras_female_03/model.sdf"
            goal.model_pose = Pose()
            goal.model_pose.position.x = 1.5
            goal.model_pose.position.y = 5.5
            q = tf.transformations.quaternion_from_euler(0.011275, 0.012812, -1.149950)
            goal.model_pose.orientation.x = q[0]
            goal.model_pose.orientation.y = q[1]
            goal.model_pose.orientation.z = q[2]
            goal.model_pose.orientation.w = q[3]

            self.client.send_goal(goal)
            self.client.wait_for_result()
            response = self.client.get_result()
            rospy.loginfo("RESPONSE FROM GAZEBO : %s",str(response))


        elif mode == "G1_after_present":
            goal = SdfInGazeboGoal()
            goal.action = "delete"
            goal.model_name = "Guest_1"
            self.client.send_goal(goal)
            self.client.wait_for_result()
            response = self.client.get_result()
            rospy.loginfo("RESPONSE FROM GAZEBO : %s",str(response))

            goal = SdfInGazeboGoal()
            goal.action = "spawn"
            goal.model_name = "Guest_1"
            goal.path_to_sdf = "/home/student/Bureau/global_palbator/src/Palbator_simulation/pmb2_simulation/pmb2_gazebo/models/citizen_extras_female_03/model.sdf"
            goal.model_pose = Pose()
            goal.model_pose.position.x = -0.5
            goal.model_pose.position.y = 6.1
            self.client.send_goal(goal)
            self.client.wait_for_result()
            response = self.client.get_result()
            rospy.loginfo("RESPONSE FROM GAZEBO : %s",str(response))

        elif mode == "G2_entrance":
            goal = SdfInGazeboGoal()
            goal.action = "spawn"
            goal.model_name = "Guest_2"
            goal.path_to_sdf = "/home/student/Bureau/global_palbator/src/Palbator_simulation/pmb2_simulation/pmb2_gazebo/models/citizen_extras_female_02/model.sdf"
            goal.model_pose = Pose()
            goal.model_pose.position.x = -0.2
            goal.model_pose.position.y = -5.5
            q = tf.transformations.quaternion_from_euler(0.011275, 0.012812, -3.088)
            goal.model_pose.orientation.x = q[0]
            goal.model_pose.orientation.y = q[1]
            goal.model_pose.orientation.z = q[2]
            goal.model_pose.orientation.w = q[3]

            self.client.send_goal(goal)
            self.client.wait_for_result()
            response = self.client.get_result()
            rospy.loginfo("RESPONSE FROM GAZEBO : %s",str(response))


        elif mode == "G2_before_present":
            goal = SdfInGazeboGoal()
            goal.action = "delete"
            goal.model_name = "Guest_2"
            self.client.send_goal(goal)
            self.client.wait_for_result()
            response = self.client.get_result()
            rospy.loginfo("RESPONSE FROM GAZEBO : %s",str(response))

            goal = SdfInGazeboGoal()
            goal.action = "spawn"
            goal.model_name = "Guest_2"
            goal.path_to_sdf = "/home/student/Bureau/global_palbator/src/Palbator_simulation/pmb2_simulation/pmb2_gazebo/models/citizen_extras_female_02/model.sdf"
            goal.model_pose = Pose()
            goal.model_pose.position.x = 1.5
            goal.model_pose.position.y = 5.5
            q = tf.transformations.quaternion_from_euler(0.011275, 0.012812, -1.149950)
            goal.model_pose.orientation.x = q[0]
            goal.model_pose.orientation.y = q[1]
            goal.model_pose.orientation.z = q[2]
            goal.model_pose.orientation.w = q[3]

            self.client.send_goal(goal)
            self.client.wait_for_result()
            response = self.client.get_result()
            rospy.loginfo("RESPONSE FROM GAZEBO : %s",str(response))


if __name__ == "__main__":
    a=Test()