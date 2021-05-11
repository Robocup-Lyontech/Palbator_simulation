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


class MoveitColumnController:

    def __init__(self, column_parameters):
        """
        Initializes the Palbator Moveit column controller.
        """
        moveit_commander.roscpp_initialize(sys.argv)

        self._parameters = column_parameters
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(self._parameters['Palbator_column_move_group'])

        self.display_trajectory_publisher = rospy.Publisher(
            self._parameters['display_column_planned_path_topic'], moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        # self.minimum_column = 0.35 + 0.06 #offset for gazebo

        self.minimum_column = self._parameters['minimum_height']
        self.maximum_column = self._parameters['maximum_height']
        self.allow_wrong_execution = self._parameters['Allow_wrong_execution']
        rospy.logwarn("{class_name} : COLUMN CONTROLLER ON".format(class_name=self.__class__.__name__))
        

    def move_column_to_pose(self, pose_name):
        """
        Moves the column in a predefined position during Moveit package's configuration giving its name.
        :param pose_name: name of the position to reach
        :type pose_name: string
        """
        rospy.loginfo("{class_name} : Column move request to position %s".format(class_name=self.__class__.__name__), pose_name)
        self.group.set_named_target(pose_name)

        plan = self.group.plan()
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(self.display_trajectory)
        rospy.loginfo("{class_name} : Moving column".format(class_name=self.__class__.__name__))
        if not self.group.go(wait=True) and not self.allow_wrong_execution:
            raise Exception("Execution failed")
        rospy.loginfo("{class_name} : Column position reached".format(class_name=self.__class__.__name__))

    def move_column(self, z_target):
        """
        Moves the column in a position on Z axis.
        :param z_target: Coordinate Z of the point to reach
        :type z_target: float
        """
        rospy.loginfo("{class_name} : Column move request to coordinate Z %s".format(class_name=self.__class__.__name__), str(z_target))
        offset = self.group.get_current_pose("palbator_arm_column").pose.position.z
        if z_target - offset < self.minimum_column:
            column_pose_target = self.minimum_column

        elif z_target - offset > self.maximum_column:
            column_pose_target = self.maximum_column

        else:
            column_pose_target = z_target - offset
        
        self.group.set_pose_reference_frame("base_footprint")
        self.group.set_joint_value_target([column_pose_target])

        plan = self.group.plan()
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(self.display_trajectory)
        rospy.loginfo("{class_name} : Moving column".format(class_name=self.__class__.__name__))
        if not self.group.go(wait=True) and not self.allow_wrong_execution:
            raise Exception("Execution failed")
        rospy.loginfo("{class_name} : Column position reached".format(class_name=self.__class__.__name__))
            