#!/usr/bin/env python
#VOIR PLUS D'INFOS https://answers.ros.org/question/246419/gazebo-spawn_model-from-py-source-code/

import rospy
from pmb2_apps.msg import SdfInGazeboAction, SdfInGazeboResult
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
import actionlib

class Test():
    def __init__(self):

        rospy.init_node("test_gazebo")

        self.action_server = actionlib.SimpleActionServer("sdf_in_gazebo_action",SdfInGazeboAction,self.sdf_in_gazebo_callback, False)


        self.delete_client = rospy.ServiceProxy("/gazebo/delete_model",DeleteModel)
        rospy.wait_for_service("/gazebo/delete_model")

        self.service_client = rospy.ServiceProxy("/gazebo/spawn_sdf_model",SpawnModel)
        rospy.wait_for_service("/gazebo/spawn_sdf_model")

        rospy.logwarn("CONNECTED")

        self.action_server.start()

        

    def sdf_in_gazebo_callback(self,goal):
        success = False

        if goal.action == "spawn":
            try:

                model_name = goal.model_name
                # with open("/home/student/Bureau/global_palbator/src/Palbator_simulation/pmb2_robot/pmb2_description/robots/pmb2.urdf.xacro") as f:
                with open(goal.path_to_sdf) as f:
                    model_xml = f.read()

                robot_namespace = goal.model_namespace
                pose_human = goal.model_pose
                reference_frame = goal.reference_frame
                response = self.service_client(model_name,model_xml,robot_namespace,pose_human,reference_frame)

                rospy.logwarn(response)
                success = True
            except Exception as e:
                rospy.logwarn("Unable to complete the action : %s",str(e))
        
        elif goal.action == "delete":
            rospy.logwarn("DELETE GOAL REQUEST")
            try:
                model_name = goal.model_name
                response = self.delete_client(model_name)
                rospy.logwarn(response)
                success = True
            except Exception as e:
                rospy.logwarn("Unable to complete the action : %s",str(e))
        
        if success:
            action_result = SdfInGazeboResult()
            action_result.gazebo_result = str(response)

            rospy.loginfo("Action succeeded")
            self.action_server.set_succeeded(action_result)
        
        else:
            rospy.loginfo("Action aborted")
            self.action_server.set_aborted()


if __name__ == "__main__":

    a=Test()
    while not rospy.is_shutdown():
        rospy.spin()