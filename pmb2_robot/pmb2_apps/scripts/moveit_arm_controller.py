#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, Constraints, OrientationConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf import TransformListener
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float32
import tf
import math
import numpy as np
from pmb2_grasp.grasp_generator import GraspGenerator
from pmb2_grasp.grasp_filter import GraspFilter


class MoveitArmController:

    MAX_RETRY = 3

    def __init__(self, arm_parameters):
        """
        Initializes the Palbator Moveit arm controller.
        """
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        self._parameters = arm_parameters
        self.target_name = "target"

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("pmb2_arm_column")

        self.display_trajectory_publisher = rospy.Publisher(
            self._parameters['display_arm_planned_path_topic'], DisplayTrajectory, queue_size=10)
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group_arm/display_planned_path", DisplayTrajectory, queue_size=10)
        self._tflistener = TransformListener()
        self.arm_hold_length = self._parameters['Palbator_hold_length']
        self.arm_min_length = self._parameters['Palbator_min_length']
        self.arm_max_length = self._parameters['Palbator_max_length']
        self.shoulder_min_rot = self._parameters['Shoulder_min_rot']
        self.shoulder_max_rot = self._parameters['Shoulder_max_rot']
        self.allow_wrong_execution = self._parameters['Allow_wrong_execution']
        rospy.logwarn("{class_name} : ARM CONTROLLER ON".format(class_name=self.__class__.__name__))

    def __display_plan(self, plan):
        self.display_trajectory = DisplayTrajectory()

        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(self.display_trajectory)

    def move_arm_to_pose(self, pose_name):
        """
        Moves the arm in a predefined position during Moveit package's configuration giving its name.
        :param pose_name: name of the position to reach
        :type pose_name: string
        """
        rospy.loginfo("{class_name} : Move arm request to position %s".format(
            class_name=self.__class__.__name__), pose_name)
        self.group.set_named_target(pose_name)

        plan = self.group.plan()

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        self.__display_plan(plan)
        rospy.loginfo("{class_name} : Moving arm ...".format(class_name=self.__class__.__name__))
        if not self.group.execute(plan, wait=True) and not self.allow_wrong_execution:
            raise Exception("Execution failed")
        rospy.loginfo("{class_name} : Arm position reached".format(class_name=self.__class__.__name__))

    def move_arm(self, goal):
        rospy.loginfo("{class_name} : Move arm request to position %s".format(
            class_name=self.__class__.__name__), goal.pose)

        self.group.set_pose_reference_frame("base_footprint")
        self.group.set_position_target(goal.pose)

        plan = self.group.plan()

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        self.__display_plan(plan)
        rospy.loginfo("{class_name} : Moving arm ...".format(class_name=self.__class__.__name__))
        if not self.group.execute(plan, wait=True) and not self.allow_wrong_execution:
            raise Exception("Execution failed")
        rospy.loginfo("{class_name} : Arm position reached".format(class_name=self.__class__.__name__))

    def look_at(self, goal):
        rospy.loginfo("{class_name} : Move arm request to look at %s".format(
            class_name=self.__class__.__name__), goal.pose)

        self._tflistener.waitForTransform("/map", "/palbator_arm_shoulder_link2", rospy.Time(0), rospy.Duration(5))
        goalTransformed = self._tflistener.transformPose("palbator_arm_shoulder_link2", goal)

        jointNames = self.group.get_joints()
        jointValues = self.group.get_current_joint_values()
        jointsDict = dict(zip(jointNames, jointValues))

        rotation = -np.arctan(goalTransformed.pose.position.x/goalTransformed.pose.position.y)

        jointsDict["palbator_arm_shoulder_2_joint"] -= rotation
        jointsDict["palbator_arm_wrist_1_joint"] += rotation

        self.group.set_joint_value_target(jointsDict)
        plan = self.group.plan()

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        self.__display_plan(plan)

        if not self.group.execute(plan, wait=True) and not self.allow_wrong_execution:
            raise Exception("Execution failed")

        rospy.loginfo("{class_name} : Arm position reached".format(class_name=self.__class__.__name__))

    def point_at(self, goal):
        rospy.loginfo("{class_name} : Move arm request to point %s".format(
            class_name=self.__class__.__name__), goal.pose)

        self.move_arm_to_pose("pointing_pose")

        self.group.set_pose_reference_frame("base_footprint")

        arm_pose = Pose()
        alpha = np.arctan(goal.pose.position.y/goal.pose.position.x)
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, alpha)
        position_effector = self.group.get_current_pose("palbator_arm_column_link")

        arm_pose.position.x = np.cos(alpha)*self.arm_hold_length
        arm_pose.position.y = np.sin(alpha)*self.arm_hold_length
        arm_pose.position.z = position_effector.pose.position.z

        arm_pose.orientation.x = q[0]
        arm_pose.orientation.y = q[1]
        arm_pose.orientation.z = q[2]
        arm_pose.orientation.w = q[3]

        constraints = Constraints()
        constraints.name = "keepLevel"
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_footprint"
        orientation_constraint.link_name = self.group.get_end_effector_link()
        orientation_constraint.orientation.x = 0.0
        orientation_constraint.orientation.y = 0.0
        orientation_constraint.orientation.z = 0.0
        orientation_constraint.orientation.w = 1.0
        orientation_constraint.absolute_x_axis_tolerance = 0.4
        orientation_constraint.absolute_y_axis_tolerance = 0.4
        orientation_constraint.absolute_z_axis_tolerance = 3.14  # ignore this axis
        orientation_constraint.weight = 1
        constraints.orientation_constraints.append(orientation_constraint)

        self.group.set_path_constraints(constraints)

        self.group.set_pose_target(arm_pose)
        plan = self.group.plan()
        self.group.clear_path_constraints()

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        self.__display_plan(plan)
        rospy.loginfo("{class_name} : Moving arm ...".format(class_name=self.__class__.__name__))
        if not self.group.execute(plan, wait=True) and not self.allow_wrong_execution:
            raise Exception("Execution failed")
        rospy.loginfo("{class_name} : Arm position reached".format(class_name=self.__class__.__name__))

    def grab(self, goal, solidPrimitive, graspParam):
        rospy.loginfo("{class_name} : Move arm request to grab object at position %s".format(
            class_name=self.__class__.__name__), goal.pose)

        target = PoseStamped()
        target.header.frame_id = "base_footprint"
        target.pose = copy.deepcopy(goal.pose)
        if solidPrimitive.type == 0:
            solidPrimitive.type = 1
            solidPrimitive.dimensions = [0.04, 0.04, 0.04]

        # floor = PoseStamped()
        # floor.header.frame_id = "base_footprint"
        # floor.pose = copy.deepcopy(goal.pose)
        # floor.pose.position.z -= solidPrimitive.dimensions[2]/2 + 0.02

        # self.scene.add_box("floor", floor, size=(solidPrimitive.dimensions[0] + 0.05, solidPrimitive.dimensions[1] + 0.05, 0.01))
        # rospy.sleep(2.0)  # wait for object to spawn
        self.scene.add_box(self.target_name, target, size=tuple(solidPrimitive.dimensions))
        rospy.sleep(5.0)  # wait for object to spawn

        graspGenerator = GraspGenerator()
        graspFilter = GraspFilter()

        grasps = graspGenerator.generateGrasps(graspParam["base_link"], goal.pose, graspParam)

        grasps = graspFilter.filterGrasps(grasps, self.target_name, graspParam)

        if len(grasps) == 0:
            # self.scene.remove_world_object("floor")
            self.scene.remove_world_object(self.target_name)
            raise Exception("Planning failed")

        # self.group.set_support_surface_name("floor")
        if 1 != self.group.pick(self.target_name, grasps[0]) and not self.allow_wrong_execution:
            # self.scene.remove_world_object("floor")
            self.scene.remove_world_object(self.target_name)
            raise Exception("Execution failed")

        self.group.set_named_target("hold_object")

        plan = self.group.plan()
        execution = self.group.execute(plan, wait=True)
        
        nbrRetry = 0
        while (not plan.joint_trajectory.points or not execution) and self.MAX_RETRY > nbrRetry:
            self.group.set_named_target("hold_object")

            plan = self.group.plan()
            execution = self.group.execute(plan, wait=True)
            nbrRetry += 1
        
        # self.scene.remove_world_object("floor")
        if self.MAX_RETRY <= nbrRetry:
            raise Exception("Execution failed")


    def drop(self, goal):
        rospy.loginfo("{class_name} : Move arm request to drop object at position %s".format(
            class_name=self.__class__.__name__), goal.pose)

        self.group.set_pose_reference_frame("base_footprint")
        arm_pose = Pose()
        alpha = np.arctan(goal.pose.position.y/goal.pose.position.x)
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, alpha)

        arm_pose.position.x = goal.pose.position.x
        arm_pose.position.y = goal.pose.position.y
        arm_pose.position.z = goal.pose.position.z

        arm_pose.orientation.x = q[0]
        arm_pose.orientation.y = q[1]
        arm_pose.orientation.z = q[2]
        arm_pose.orientation.w = q[3]

        self.group.set_pose_target(arm_pose)

        plan= self.group.plan()

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        if not self.group.execute(plan, wait=True) and not self.allow_wrong_execution:
            raise Exception("Execution failed")

    def removeAttachedObject(self, goal):
        self.group.detach_object(self.target_name)
        self.scene.remove_world_object(self.target_name)
        arm_pose = Pose()
        alpha = np.arctan(goal.pose.position.y/goal.pose.position.x)
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, alpha)

        arm_pose.position.x = goal.pose.position.x - 0.05
        arm_pose.position.y = goal.pose.position.y
        arm_pose.position.z = goal.pose.position.z

        arm_pose.orientation.x = q[0]
        arm_pose.orientation.y = q[1]
        arm_pose.orientation.z = q[2]
        arm_pose.orientation.w = q[3]

        self.group.set_pose_target(arm_pose)

        plan= self.group.plan()

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        if not self.group.execute(plan, wait=True) and not self.allow_wrong_execution:
            raise Exception("Execution failed")

    def move_column(self, goal):
        rospy.loginfo("{class_name} : Move column to %s".format(
            class_name=self.__class__.__name__), goal)

        self._tflistener.waitForTransform("/map", "/palbator_arm_column_link", rospy.Time(0), rospy.Duration(5))
        (trans,rot) = self._tflistener.lookupTransform("map", "palbator_arm_column_link", rospy.Time(0))
        rospy.loginfo("{class_name} : Move column to %s".format(
            class_name=self.__class__.__name__), trans)

        jointNames = self.group.get_joints()
        jointValues = self.group.get_current_joint_values()
        jointsDict = dict(zip(jointNames, jointValues))

        jointsDict["palbator_arm_column_joint"] += goal - trans[2]

        self.group.set_joint_value_target(jointsDict)
        plan = self.group.plan()

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        self.__display_plan(plan)

        if not self.group.execute(plan, wait=True) and not self.allow_wrong_execution:
            raise Exception("Execution failed")

        rospy.loginfo("{class_name} : Arm position reached".format(class_name=self.__class__.__name__))

    def namo(self, goal):
        rospy.loginfo("{class_name} : Move arm request to move object at position %s".format(
            class_name=self.__class__.__name__), goal.pose)

        self.group.set_pose_reference_frame("base_footprint")
        namoArea = PoseStamped()
        namoArea.header.frame_id = "base_footprint"
        namoArea.pose.position.z = -0.005
        namoArea.pose.orientation.w = 1.0
        self.scene.add_box("namoArea", namoArea, size=(1, 1, 0.01))

        arm_pose = Pose()
        alpha = np.arctan(goal.pose.position.y/goal.pose.position.x)
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, alpha)

        arm_pose.position.x = goal.pose.position.x
        arm_pose.position.y = goal.pose.position.y
        arm_pose.position.z = 0.044

        arm_pose.orientation.x = q[0]
        arm_pose.orientation.y = q[1]
        arm_pose.orientation.z = q[2]
        arm_pose.orientation.w = q[3]

        self.group.set_pose_target(arm_pose)

        plan = self.group.plan()

        if not plan.joint_trajectory.points:
            self.scene.remove_world_object("namoArea")
            raise Exception("Planning failed")

        if not self.group.execute(plan, wait=True) and not self.allow_wrong_execution:
            self.scene.remove_world_object("namoArea")
            raise Exception("Execution failed")

        jointNames = self.group.get_joints()
        jointValues = self.group.get_current_joint_values()
        jointsDict = dict(zip(jointNames, jointValues))

        jointsDict["palbator_arm_shoulder_1_joint"] -= 0.785398

        self.group.set_joint_value_target(jointsDict)
        plan = self.group.plan()

        if not plan.joint_trajectory.points:
            self.scene.remove_world_object("namoArea")
            raise Exception("Planning failed")

        self.__display_plan(plan)

        if not self.group.execute(plan, wait=True) and not self.allow_wrong_execution:
            self.scene.remove_world_object("namoArea")
            raise Exception("Execution failed")

        rospy.loginfo("{class_name} : Namo done".format(class_name=self.__class__.__name__))

        self.scene.remove_world_object("namoArea")
