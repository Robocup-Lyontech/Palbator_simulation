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
import math
import numpy as np


class MoveitArmController:

    def __init__(self, arm_parameters):
        """
        Initializes the Palbator Moveit arm controller.
        """
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        self._parameters = arm_parameters

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(self._parameters['Palbator_arm_move_group'])

        self.display_trajectory_publisher = rospy.Publisher(
            self._parameters['display_arm_planned_path_topic'], moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self._tflistener = TransformListener()
        self.arm_length = self._parameters['Palbator_arm_length']

        rospy.logwarn("{class_name} : ARM CONTROLLER ON".format(class_name=self.__class__.__name__))

    def move_arm_to_pose(self, pose_name):
        """
        Moves the arm in a predefined position during Moveit package's configuration giving its name.
        :param pose_name: name of the position to reach
        :type pose_name: string
        """
        rospy.loginfo("{class_name} : Move arm request to position %s".format(class_name=self.__class__.__name__), pose_name)
        self.group.set_named_target(pose_name)

        plan1 = self.group.plan()
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan1)
        self.display_trajectory_publisher.publish(self.display_trajectory)
        rospy.loginfo("{class_name} : Moving arm ...".format(class_name=self.__class__.__name__))
        # rospy.sleep(5)
        self.group.go(wait=True)
        rospy.loginfo("{class_name} : Arm position reached".format(class_name=self.__class__.__name__))

    def move_arm(self, x_target, y_target, z_target, mode="normal"):
        """
        Moves the arm end effector in a position to point the target of coordinates x_target and y_target.
        :param x_target: Coordinate X of the target to point
        :type x_target: float
        :param y_target: Coordinate Y of the target to point
        :type y_target: float
        :param mode: "normal", "pre_grasp": close to the body or "cartesian": using straight line.
        :type mode: string
        """
        rospy.loginfo("{class_name} : Move arm request to point target x: %s y: %s".format(
            class_name=self.__class__.__name__), str(x_target), str(y_target))
        pose_target = self.calculate_pose(x_target, y_target, z_target, mode)

        self.group.set_pose_reference_frame("base_footprint")

        if mode == "cartesian":
            (plan, fraction) = self.group.compute_cartesian_path(
                [pose_target], 0.01, 0.0)

        else:
            self.group.set_pose_target(pose_target)

            plan1 = self.group.plan()
            self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

            self.display_trajectory.trajectory_start = self.robot.get_current_state()
            self.display_trajectory.trajectory.append(plan1)
            self.display_trajectory_publisher.publish(self.display_trajectory)
        rospy.loginfo("{class_name} : Moving arm ...".format(class_name=self.__class__.__name__))
        if mode == "cartesian":
            self.group.execute(plan)
        else:
            self.group.go(wait=True)
        rospy.loginfo("{class_name} : Arm position reached".format(class_name=self.__class__.__name__))

    def calculate_pose(self, x_target, y_target, z_target, mode="normal"):
        """
        Calculate the position and the orientation the arm end effector has to reach to point the target of coordinates x_target and y_target.
        Returns the Pose message containing the position and the orientation the arm has to reach.
        :param x_target: Coordinate X of the target to point
        :type x_target: float
        :param y_target: Coordinate Y of the target to point
        :type y_target: float
        :param mode: "normal", "pre_grasp": close to the body or "cartesian": using straight line.
        :type mode: string
        """
        arm_pose = Pose()

        alpha = np.arctan(y_target/x_target)

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, alpha)
        position_effector = self.group.get_current_pose("palbator_arm_shoulder_link1")

        if mode == "normal":
            x_arm = x_target
            y_arm = y_target
            arm_pose.position.z = position_effector.pose.position.z
        elif mode == "pre_grasp":
            x_arm = np.cos(alpha)*self.arm_length
            y_arm = np.sin(alpha)*self.arm_length
            arm_pose.position.z = position_effector.pose.position.z
        elif mode == "cartesian":
            x_arm = x_target
            y_arm = y_target
            arm_pose.position.z = z_target

        else:
            rospy.logerr("{class_name} : Wrong mode %s".format(class_name=self.__class__.__name__), mode)
            return Pose()

        arm_pose.position.x = x_arm
        arm_pose.position.y = y_arm

        arm_pose.orientation.x = q[0]
        arm_pose.orientation.y = q[1]
        arm_pose.orientation.z = q[2]
        arm_pose.orientation.w = q[3]

        return arm_pose
