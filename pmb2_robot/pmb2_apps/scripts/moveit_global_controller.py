#!/usr/bin/env python
import rospy
from moveit_column_controller import MoveitColumnController
from moveit_arm_controller import MoveitArmController
from moveit_gripper_controller import MoveitGripperController
from geometry_msgs.msg import Point, PoseStamped
import moveit_commander
import sys

import actionlib
from pmb2_apps.msg import ArmControlAction, ArmControlResult

from tf import TransformListener
from geometry_msgs.msg import PointStamped
import json
import math


class MoveitGlobalController:

    def __init__(self):
        """
        Initializes the global controller which will control Palbator Moveit arm and column controllers
        """
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_global_controller', anonymous=True)
        self.scene = moveit_commander.PlanningSceneInterface()

        if rospy.has_param("~Palbator_column_parameters"):
            column_parameters = rospy.get_param("~Palbator_column_parameters")
            self._column_controller = MoveitColumnController(column_parameters)
        else:
            rospy.logerr("{class_name} : No parameters specified for Moveit Palbator Column controller. Can't start column controller.")

        if rospy.has_param("~Palbator_arm_parameters"):
            arm_parameters = rospy.get_param("~Palbator_arm_parameters")
            self._arm_controller = MoveitArmController(arm_parameters)
        else:
            rospy.logerr("{class_name} : No parameters specified for Moveit Palbator Arm controller. Can't start arm controller.")

        if rospy.has_param("~Palbator_gripper_parameters"):
            gripper_parameters = rospy.get_param("~Palbator_gripper_parameters")
            self._gripper_controller = MoveitGripperController(gripper_parameters)
        else:
            rospy.logerr("{class_name} : No parameters specified for Moveit Palbator Gripper controller. Can't start gripper controller.")

        self._column_controller.move_column_to_pose("travelling_pose")
        self._arm_controller.move_arm_to_pose("travelling_pose")
        self._gripper_controller.move_gripper_to_pose("close_gripper")

        if rospy.has_param("~Moveit_global_controller_action_name"):
            action_server_name = rospy.get_param("~Moveit_global_controller_action_name")
            self._arm_control_server = actionlib.SimpleActionServer(action_server_name, ArmControlAction, self.executeActionServer, False)
            self._arm_control_server.start()
        else:
            rospy.logerr("{class_name} : No name specified for Global Moveit Palbator controller action server. Can't start action server.")

        rospy.loginfo("{class_name} : Global Palbator Moveit Controller initialized".format(class_name=self.__class__.__name__))

    def executeActionServer(self, goal):
        """
        Action Server callback for Moveit global control. Can point an object or move in a defined position to travel without risks.
        :param goal: contains action data to do
        :type goal: ArmControlGoal
        """
        isActionSucceed = False
        action_result = ArmControlResult()
        if "Grasping" in goal.action or "Pointing" in goal.action or "Droping" in goal.action:
            rospy.loginfo("{class_name} : Received %s action goal".format(class_name=self.__class__.__name__), goal.action)
            listener = TransformListener()
            now = rospy.Time(0)

            if goal.action == 'Grasping' or goal.action == 'Pointing':
                # ToDo test
                listener.waitForTransform("map", goal.object_label, now, rospy.Duration(20))
                (trans, rot) = listener.lookupTransform("map", goal.object_label, now)

                object_point = PointStamped()
                object_point.header.frame_id = "map"
                object_point.header.stamp = now
                object_point.point.x = trans[0]
                object_point.point.y = trans[1]
                object_point.point.z = trans[2]

            elif goal.action == 'GraspingXYZ' or goal.action == 'PointingXYZ' or "Droping" in goal.action:
                object_point = PointStamped()
                object_point.header.frame_id = "map"
                object_point.header.stamp = now
                object_point.point.x = goal.coord_x
                object_point.point.y = goal.coord_y
                object_point.point.z = goal.coord_z

            rospy.loginfo("{class_name} : Goal coords in map : %s".format(class_name=self.__class__.__name__), str(object_point))
            listener.waitForTransform("/map", "/base_footprint", now, rospy.Duration(20))
            target = listener.transformPoint("base_footprint", object_point)

            rospy.loginfo("{class_name} : Goal coords in base_footprint : %s".format(class_name=self.__class__.__name__), str(target))

            target_x = target.point.x
            target_y = target.point.y
            target_z = target.point.z

            alpha = math.atan2(target_y, target_x)

            rospy.logwarn("{class_name} : ANGLE TO GOAL %s Radian %s Degree".format(
                class_name=self.__class__.__name__), str(alpha), str(alpha*180/math.pi))

            if (target_x > 0):
                if (target_y > 0):
                    rotationNeeded = -math.pi/2
                else:
                    rotationNeeded = 0.0
            else:
                if (target_y > 0):
                    rotationNeeded = -math.pi
                else:
                    rotationNeeded = math.pi/2

            if (rotationNeeded != 0):
                rospy.logwarn("{class_name} : ROTATION OF %.2f RADIAN NEEDED TO POINT".format(class_name=self.__class__.__name__), rotationNeeded)

                json_result = {
                    "action": goal.action,
                    "rotation": rotationNeeded,
                    "status": 'Aborted'
                }
                action_result.action_output = json.dumps(json_result)

            else:
                if "Grasping" in goal.action:
                    # put the arm in base pose before grab
                    self._arm_controller.move_arm_to_pose("pointing_pose")

                    # align end effector and object to grab
                    self._arm_controller.move_arm(target_x, target_y, target_z, mode="pre_grasp")
                    self._column_controller.move_column(target_z)

                    self._gripper_controller.move_gripper(1.0)

                    # move end effector to the object
                    self._arm_controller.move_arm(target_x, target_y, target_z, mode="cartesian")

                    # close gripper
                    self._gripper_controller.move_gripper(0.1)

                    self._arm_controller.move_arm(target_x, target_y, target_z, mode="pre_grasp")
                    self._column_controller.move_column_to_pose("travelling_pose")

                    isActionSucceed = True

                elif "Pointing" in goal.action:
                    # align end effector and position to point
                    self._arm_controller.move_arm(target_x, target_y, target_z, mode="pre_grasp")
                    self._column_controller.move_column(target_z)

                    isActionSucceed = True

                elif "Droping" in goal.action:
                    # align end effector and position to drop
                    self._arm_controller.move_arm(target_x, target_y, target_z, mode="pre_grasp")
                    self._column_controller.move_column(target_z)

                    # move end effector to the position to drop
                    self._arm_controller.move_arm(target_x, target_y, target_z, mode="cartesian")

                    # drop
                    self._gripper_controller.move_gripper(1.0)

                    isActionSucceed = True

                json_result = {
                    "action": goal.action,
                    "status": ('Success' if isActionSucceed else 'Aborted')
                }
                action_result.action_output = json.dumps(json_result)

        elif goal.action == 'Travelling':
            rospy.loginfo("{class_name} : Received travelling action goal".format(class_name=self.__class__.__name__))
            self._column_controller.move_column_to_pose("travelling_pose")
            self._arm_controller.move_arm_to_pose("travelling_pose")
            self._gripper_controller.move_gripper_to_pose("close_gripper")
            json_result = {
                "action": goal.action,
                "status": 'Success'
            }
            action_result.action_output = json.dumps(json_result)
            isActionSucceed = True

        else:
            rospy.logwarn("{class_name} : unable to find or launch function corresponding to the action %s".format(
                class_name=self.__class__.__name__), str(goal.action))

        if isActionSucceed:
            rospy.loginfo("{class_name} : Action %s succeeded".format(class_name=self.__class__.__name__), goal.action)
            self._arm_control_server.set_succeeded(action_result)
        else:
            rospy.loginfo("{class_name} : Action %s aborted".format(class_name=self.__class__.__name__), goal.action)
            self._arm_control_server.set_aborted()
        return


if __name__ == "__main__":

    global_controller = MoveitGlobalController()
    while not rospy.is_shutdown():
        rospy.spin()
