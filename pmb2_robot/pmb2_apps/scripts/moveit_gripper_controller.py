#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from tf import TransformListener
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
import tf


class MoveitGripperController:

    def __init__(self, gripper_parameters):
        """
        Initializes the Palbator Moveit gripper controller.
        """
        moveit_commander.roscpp_initialize(sys.argv)

        self._parameters = gripper_parameters
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(self._parameters['Palbator_gripper_move_group'])

        self.display_trajectory_publisher = rospy.Publisher(
            self._parameters['display_gripper_planned_path_topic'], moveit_msgs.msg.DisplayTrajectory, queue_size=10)

        self.minimum_gripper = self._parameters['minimum_opening']
        self.maximum_gripper = self._parameters['maximum_opening']
        self.allow_wrong_execution = self._parameters['Allow_wrong_execution']
        rospy.logwarn("{class_name} : GRIPPER CONTROLLER ON".format(class_name=self.__class__.__name__))

    def move_gripper_to_pose(self, pose_name):
        """
        Moves the gripper in a predefined position during Moveit package's configuration giving its name.
        :param pose_name: name of the position to reach
        :type pose_name: string
        """
        rospy.loginfo("{class_name} : Gripper move request to position %s".format(class_name=self.__class__.__name__), pose_name)
        self.group.set_named_target(pose_name)

        plan = self.group.plan()
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(self.display_trajectory)
        rospy.loginfo("{class_name} : Moving gripper".format(class_name=self.__class__.__name__))
        if not self.group.go(wait=True) and not self.allow_wrong_execution:
            raise Exception("Execution failed")
        rospy.loginfo("{class_name} : Gripper position reached".format(class_name=self.__class__.__name__))

    def move_gripper(self, opening_size):
        """
        Set the gripper opening in percent.
        :param opening_size: Opening size value from 0.0 to 1.0
        :type opening_size: float
        """
        rospy.loginfo("{class_name} : Gripper opening request %s%%".format(class_name=self.__class__.__name__), str(opening_size * 100))

        open_value = opening_size * (self.maximum_gripper - self.minimum_gripper) + self.minimum_gripper

        self.group.set_pose_reference_frame("base_footprint")
        self.group.set_joint_value_target([open_value, open_value])

        plan = self.group.plan()
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(self.display_trajectory)
        rospy.loginfo("{class_name} : Moving gripper".format(class_name=self.__class__.__name__))
        if not self.group.go(wait=True) and not self.allow_wrong_execution:
            raise Exception("Execution failed")
        rospy.loginfo("{class_name} : Gripper position reached".format(class_name=self.__class__.__name__))
