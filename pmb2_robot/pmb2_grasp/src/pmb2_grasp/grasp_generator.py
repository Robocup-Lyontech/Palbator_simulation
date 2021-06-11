#!/usr/bin/env python
import rospy
import math
from moveit_msgs.msg import Grasp
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import copy


class GraspGenerator:
    X_AXIS = 0
    Y_AXIS = 1
    Z_AXIS = 2

    def __init__(self):
        pass

    def generateGrasps(self, baseLink, objectPose, graspParam):
        self.grasps = []

        self._gripperJoints = graspParam["joints"]
        self._pregrasp_posture = graspParam["pregrasp_posture"]
        self._grasp_posture = graspParam["grasp_posture"]
        self._eef = graspParam["end_effector_name"]
        self._eef_approach_vector = graspParam["eef_approach_vector"]
        self._eef_retreat_vector = graspParam["eef_retreat_vector"]
        self._approach_dist = graspParam["approach_distance_desired"]
        self._min_approach_dist = graspParam["min_approach_distance"]
        self._lift_dist = graspParam["lift_distance_desired"]
        self._min_lift_dist = graspParam["min_lift_distance"]

        rospy.loginfo("{class_name} : Generating grasps".format(class_name=self.__class__.__name__))

        # add grasp for every face
        self.addFaceGrasps(baseLink, objectPose, self.X_AXIS)
        self.addFaceGrasps(baseLink, objectPose, self.Y_AXIS)
        self.addFaceGrasps(baseLink, objectPose, self.Z_AXIS)

        rospy.loginfo("{class_name} : Generated %d possible grasps".format(
            class_name=self.__class__.__name__), len(self.grasps))
        return self.grasps

    def addFaceGrasps(self, baseLink, objectPose, axis):
        graspPose = Pose()

        if axis == self.X_AXIS:
            graspOrientation = [objectPose.orientation.x, objectPose.orientation.y,
                                objectPose.orientation.z, objectPose.orientation.w]
            for side in range(2):
                graspOrientation = quaternion_multiply(graspOrientation, quaternion_from_euler(0, 0, math.pi * side))
                for rotationNumber in range(4):
                    graspPose.position = copy.deepcopy(objectPose.position)
                    q = quaternion_multiply(graspOrientation, quaternion_from_euler(
                        (rotationNumber - 1) * math.pi / 2, 0, 0))
                    graspPose.orientation.x = q[0]
                    graspPose.orientation.y = q[1]
                    graspPose.orientation.z = q[2]
                    graspPose.orientation.w = q[3]
                    self.addGrasp(baseLink, graspPose)

        if axis == self.Y_AXIS:
            graspOrientation = [objectPose.orientation.x, objectPose.orientation.y,
                                objectPose.orientation.z, objectPose.orientation.w]
            graspOrientation = quaternion_multiply(graspOrientation, quaternion_from_euler(0, 0, math.pi/2))
            for side in range(2):
                graspOrientation = quaternion_multiply(graspOrientation, quaternion_from_euler(0, 0, math.pi * side))
                for rotationNumber in range(4):
                    graspPose.position = copy.deepcopy(objectPose.position)
                    q = quaternion_multiply(graspOrientation, quaternion_from_euler(
                        rotationNumber * math.pi / 2, 0, 0))
                    graspPose.orientation.x = q[0]
                    graspPose.orientation.y = q[1]
                    graspPose.orientation.z = q[2]
                    graspPose.orientation.w = q[3]
                    self.addGrasp(baseLink, graspPose)

        if axis == self.Z_AXIS:
            graspOrientation = [objectPose.orientation.x, objectPose.orientation.y,
                                objectPose.orientation.z, objectPose.orientation.w]
            graspOrientation = quaternion_multiply(graspOrientation, quaternion_from_euler(0, math.pi/2, 0))
            for side in range(2):
                graspOrientation = quaternion_multiply(graspOrientation, quaternion_from_euler(0, 0, math.pi * side))
                for rotationNumber in range(4):
                    graspPose.position = copy.deepcopy(objectPose.position)
                    q = quaternion_multiply(graspOrientation, quaternion_from_euler(
                        rotationNumber * math.pi / 2, 0, 0))
                    graspPose.orientation.x = q[0]
                    graspPose.orientation.y = q[1]
                    graspPose.orientation.z = q[2]
                    graspPose.orientation.w = q[3]
                    self.addGrasp(baseLink, graspPose)

    def addGrasp(self, baseLink, graspPose):
        grasp = Grasp()

        grasp.id = str(len(self.grasps))
        grasp.grasp_pose.header.frame_id = baseLink
        grasp.grasp_pose.pose = copy.deepcopy(graspPose)

        grasp.pre_grasp_approach.direction.header.frame_id = self._eef
        grasp.pre_grasp_approach.direction.vector.x = self._eef_approach_vector[0]
        grasp.pre_grasp_approach.direction.vector.y = self._eef_approach_vector[1]
        grasp.pre_grasp_approach.direction.vector.z = self._eef_approach_vector[2]
        grasp.pre_grasp_approach.min_distance = self._min_approach_dist
        grasp.pre_grasp_approach.desired_distance = self._approach_dist

        grasp.post_grasp_retreat.direction.header.frame_id = self._eef
        grasp.post_grasp_retreat.direction.vector.x = self._eef_retreat_vector[0]
        grasp.post_grasp_retreat.direction.vector.y = self._eef_retreat_vector[1]
        grasp.post_grasp_retreat.direction.vector.z = self._eef_retreat_vector[2]
        grasp.post_grasp_retreat.min_distance = self._min_lift_dist
        grasp.post_grasp_retreat.desired_distance = self._lift_dist

        points = JointTrajectoryPoint()
        for i in range(len(self._gripperJoints)):
            grasp.pre_grasp_posture.joint_names.append(self._gripperJoints[i])
            points.positions.append(self._pregrasp_posture[i])
        points.time_from_start += rospy.rostime.Duration(5.0)
        grasp.pre_grasp_posture.points = [copy.deepcopy(points)]

        points = JointTrajectoryPoint()
        for i in range(len(self._gripperJoints)):
            grasp.grasp_posture.joint_names.append(self._gripperJoints[i])
            points.positions.append(self._grasp_posture[i])
        points.time_from_start += rospy.rostime.Duration(5.0)
        grasp.grasp_posture.points = [copy.deepcopy(points)]

        self.grasps.append(grasp)
