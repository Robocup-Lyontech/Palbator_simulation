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
            self._parameters['display_arm_planned_path_topic'], DisplayTrajectory, queue_size=10)
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
        rospy.loginfo("{class_name} : Move arm request to position %s".format(class_name=self.__class__.__name__), pose_name)
        self.group.set_named_target(pose_name)

        plan = self.group.plan()

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        self.__display_plan(plan)
        rospy.loginfo("{class_name} : Moving arm ...".format(class_name=self.__class__.__name__))
        if not self.group.go(wait=True) and not self.allow_wrong_execution:
            raise Exception("Execution failed")
        rospy.loginfo("{class_name} : Arm position reached".format(class_name=self.__class__.__name__))

    def move_arm(self, goal):
        rospy.loginfo("{class_name} : Move arm request to position %s".format(class_name=self.__class__.__name__), goal.pose)

        self.group.set_pose_reference_frame("base_footprint")
        self.group.set_position_target(goal.pose)

        plan = self.group.plan()

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        self.__display_plan(plan)
        rospy.loginfo("{class_name} : Moving arm ...".format(class_name=self.__class__.__name__))
        if not self.group.go(wait=True) and not self.allow_wrong_execution:
            raise Exception("Execution failed")
        rospy.loginfo("{class_name} : Arm position reached".format(class_name=self.__class__.__name__))

    def look_at(self, goal):
        rospy.loginfo("{class_name} : Move arm request to look at %s".format(class_name=self.__class__.__name__), goal.pose)

        jointNames = self.group.get_joints()

        self._tflistener.waitForTransform("/map", "/palbator_arm_shoulder_link2", rospy.Time(0), rospy.Duration(5))
        goalTransformed = self._tflistener.transformPose("palbator_arm_shoulder_link2", goal)

        jointValues = self.group.get_current_joint_values()
        jointsDict = dict(zip(jointNames, jointValues))
        
        rotation = jointsDict["palbator_arm_shoulder_1_joint"] + np.arctan(goalTransformed.pose.position.y/goalTransformed.pose.position.x) - math.pi/2

        if rotation < self.shoulder_min_rot:
            rotation = self.shoulder_min_rot
        elif rotation > self.shoulder_max_rot:
            rotation = self.shoulder_max_rot

        self.group.set_joint_value_target({"palbator_arm_shoulder_1_joint": rotation})
        plan = self.group.plan()

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        self.__display_plan(plan)

        if not self.group.execute(plan, wait=True) and not self.allow_wrong_execution:
            raise Exception("Execution failed")

        rospy.loginfo("{class_name} : Arm position reached".format(class_name=self.__class__.__name__))


    def point_at(self, goal):
        rospy.loginfo("{class_name} : Move arm request to point %s".format(class_name=self.__class__.__name__), goal.pose)

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
        orientation_constraint.absolute_z_axis_tolerance = 3.14 #ignore this axis
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
        rospy.loginfo("{class_name} : Move arm request to grab object at position %s".format(class_name=self.__class__.__name__), goal.pose)

        target = PoseStamped()
        target.header.frame_id = "base_footprint"
        target.pose = copy.deepcopy(goal.pose)
        target_name = "target"
        if solidPrimitive.type == 0:
            self.scene.add_box(target_name, target, size=(0.04, 0.04, 0.04))
        else:
            self.scene.add_box(target_name, target, size=tuple(solidPrimitive.dimensions))
        rospy.sleep(1)  # wait for object to spawn

        graspGenerator = GraspGenerator()
        graspFilter = GraspFilter()

        grasps = graspGenerator.generateGrasps(graspParam["base_link"], goal.pose, graspParam)

        grasps = graspFilter.filterGrasps(grasps, target_name, graspParam)

        if len(grasps) == 0:
            raise Exception("Planning failed")

        if 1 != self.group.pick(target_name, grasps[0]) and not self.allow_wrong_execution:
            raise Exception("Execution failed")

        # waypoints = [self.group.get_current_pose().pose]

        # waypoint = copy.deepcopy(waypoints[-1])
        # waypoint.position.z = goal.point.z
        # waypoints.append(waypoint)

        # waypoint = copy.deepcopy(waypoints[-1])
        # waypoint.position.x = goal.point.x
        # waypoint.position.y = goal.point.y
        # waypoints.append(waypoint)

        # waypoints.pop(0)

        # (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)

        # if not plan.joint_trajectory.points:
        #     raise Exception("Planning failed")

        # self.__display_plan(plan)
        # if (not self.group.execute(plan, wait=True) or fraction != 1.0) and not self.allow_wrong_execution:
        #     raise Exception("Execution failed")


    def post_grab(self, goal):
        rospy.loginfo("{class_name} : Move arm request to standby position".format(class_name=self.__class__.__name__))

        waypoints = [self.group.get_current_pose().pose]

        waypoint = copy.deepcopy(waypoints[-1])
        waypoint.position.z += 0.05
        waypoints.append(waypoint)

        waypoint = copy.deepcopy(waypoints[-1])
        alpha = np.arctan(goal.pose.position.y/goal.pose.position.x)
        waypoint.position.x = np.cos(alpha)*self.arm_hold_length
        waypoint.position.y = np.sin(alpha)*self.arm_hold_length
        waypoints.append(waypoint)

        waypoint = copy.deepcopy(waypoints[-1])
        waypoint.position.z = self.group.get_current_pose("palbator_arm_column_link").pose.position.z
        waypoints.append(waypoint)

        waypoints.pop(0)

        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        self.__display_plan(plan)
        if (not self.group.execute(plan, wait=True) or fraction != 1.0) and not self.allow_wrong_execution:
            raise Exception("Execution failed")
            

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
        orientation_constraint.absolute_z_axis_tolerance = 3.14 #ignore this axis
        orientation_constraint.weight = 1
        constraints.orientation_constraints.append(orientation_constraint)

        self.group.set_path_constraints(constraints)
        self.move_arm_to_pose("pointing_pose")
        self.group.clear_path_constraints()

    def drop(self, goal):
        rospy.loginfo("{class_name} : Move arm request to drop object at position %s".format(class_name=self.__class__.__name__), goal.pose)

        waypoints = [self.group.get_current_pose().pose]

        height = self.group.get_current_pose("palbator_arm_column_link").pose.position.z
        if goal.pose.position.z != height:
            waypoint = copy.deepcopy(waypoints[-1])
            waypoint.position.z = goal.pose.position.z
            waypoints.append(waypoint)

        waypoint = copy.deepcopy(waypoints[-1])
        waypoint.position.z += 0.05
        waypoints.append(waypoint)

        waypoint = copy.deepcopy(waypoints[-1])
        waypoint.position.x = goal.pose.position.x
        waypoint.position.y = goal.pose.position.y
        waypoints.append(waypoint)

        waypoint = copy.deepcopy(waypoints[-1])
        waypoint.position.z = goal.pose.position.z
        waypoints.append(waypoint)

        waypoints.pop(0)

        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.03, 0.0)

        if not plan.joint_trajectory.points:
            raise Exception("Planning failed")

        self.__display_plan(plan)
        if (not self.group.execute(plan, wait=True) or fraction != 1.0) and not self.allow_wrong_execution:
            raise Exception("Execution failed")
