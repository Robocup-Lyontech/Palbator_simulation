#!/usr/bin/env python
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState, DisplayTrajectory
from sensor_msgs.msg import JointState
import copy


class GraspFilter:

    def __init__(self):
        pass

    def filterGrasps(self, grasp_candidates, arm_group, hand_group, graspParam):
        self._gripperJoints = graspParam["joints"]
        self._pregrasp_posture = graspParam["pregrasp_posture"]
        self._grasp_posture = graspParam["grasp_posture"]
        self._eef_approach_vector = graspParam["eef_approach_vector"]
        
        self.filterPregrasp(grasp_candidates[0], arm_group, hand_group, graspParam)
        self.filterPregrasp(grasp_candidates[1], arm_group, hand_group, graspParam)
        self.filterPregrasp(grasp_candidates[2], arm_group, hand_group, graspParam)
        self.filterPregrasp(grasp_candidates[3], arm_group, hand_group, graspParam)

    def filterPregrasp(self, grasp_candidate, arm_group, hand_group, graspParam):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self._gripperJoints
        joint_state.position = self._pregrasp_posture
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        hand_group.set_start_state(moveit_robot_state)

        waypoints = [copy.deepcopy(grasp_candidate.grasp_pose.pose)]
        waypoints[0].position.x += self._eef_approach_vector[0] * grasp_candidate.pre_grasp_approach.min_distance
        waypoints[0].position.y += self._eef_approach_vector[1] * grasp_candidate.pre_grasp_approach.min_distance
        waypoints[0].position.z += self._eef_approach_vector[2] * grasp_candidate.pre_grasp_approach.min_distance

        waypoints.append(copy.deepcopy(grasp_candidate.grasp_pose.pose))

        (plan, fraction) = arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        rospy.loginfo("{class_name} : fraction: %.2f".format(class_name=self.__class__.__name__), fraction)
        display_trajectory = DisplayTrajectory()

        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group_arm/display_planned_path", DisplayTrajectory, queue_size=10)
        display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(5)