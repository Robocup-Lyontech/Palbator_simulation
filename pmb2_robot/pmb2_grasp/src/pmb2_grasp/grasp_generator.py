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
        self._graspXFace = True     # allow grasp on face x or -x
        self._graspYFace = True     # allow grasp on face y or -y
        self._graspZFace = True     # allow grasp on face z or -z
        self._graspEdge = True      # allow grasp on edge
        self._planCorner = True     # allow grasp on corner

    def generateGrasps(self, baseLink, objectPose, graspParam):
        """Generate Grasp data

        :param baseLink: Base link name
        :type baseLink: String
        :param objectPose: Pose of the object to grasp
        :type objectPose: geometry_msgs/Pose
        :param graspParam: Parameter of the grasp
        :type graspParam: Dict
        :return: List of possible grasp (not taking feasibility into account)
        :rtype: moveit_msgs/Grasp[]
        """
        self.grasps = []

        self._gripperJoints = graspParam["joints"]
        self._pregrasp_posture = graspParam["pregrasp_posture"]
        self._grasp_posture = graspParam["grasp_posture"]
        self._eef = graspParam["end_effector_name"]
        self._eef_approach_vector = graspParam["eef_approach_vector"]
        self._eef_retreat_vector = graspParam["eef_retreat_vector"]
        self._eef_to_transform = graspParam["eef_to_transform"]
        self._approach_dist = graspParam["approach_distance_desired"]
        self._min_approach_dist = graspParam["min_approach_distance"]
        self._lift_dist = graspParam["lift_distance_desired"]
        self._min_lift_dist = graspParam["min_lift_distance"]

        rospy.loginfo("{class_name} : Generating grasps".format(class_name=self.__class__.__name__))

        # add grasp for every face
        if self._graspXFace:
            self.addFaceGrasps(baseLink, objectPose, self.X_AXIS)
        if self._graspYFace:
            self.addFaceGrasps(baseLink, objectPose, self.Y_AXIS)
        if self._graspZFace:
            self.addFaceGrasps(baseLink, objectPose, self.Z_AXIS)

        if self._graspEdge:
            self.addEdgeGrasps(baseLink, objectPose)

        rospy.loginfo("{class_name} : Generated %d possible grasps".format(
            class_name=self.__class__.__name__), len(self.grasps))
        return self.grasps

    def addFaceGrasps(self, baseLink, objectPose, axis):
        """Generate face oriented grasps

        :param baseLink: Base link name
        :type baseLink: String
        :param objectPose: Pose of the object to grasp
        :type objectPose: geometry_msgs/Pose
        :param axis: Axis corresponding to the face to add grasp to
        :type axis: Int
        """
        graspPose = Pose()

        if axis == self.X_AXIS:
            for side in range(2):
                graspOrientation = [objectPose.orientation.x, objectPose.orientation.y,
                                    objectPose.orientation.z, objectPose.orientation.w]
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

        if axis == self.Y_AXIS:
            for side in range(2):
                graspOrientation = [objectPose.orientation.x, objectPose.orientation.y,
                                    objectPose.orientation.z, objectPose.orientation.w]
                graspOrientation = quaternion_multiply(graspOrientation, quaternion_from_euler(0, 0, math.pi/2))
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
            for side in range(2):
                graspOrientation = [objectPose.orientation.x, objectPose.orientation.y,
                                    objectPose.orientation.z, objectPose.orientation.w]
                graspOrientation = quaternion_multiply(graspOrientation, quaternion_from_euler(0, math.pi/2, 0))
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

    def addEdgeGrasps(self, baseLink, objectPose):
        """Generate edge oriented grasps

        :param baseLink: Base link name
        :type baseLink: String
        :param objectPose: Pose of the object to grasp
        :type objectPose: geometry_msgs/Pose
        """
        graspPose = Pose()

        # select corner connected to x and y faces
        for corner in range(4):
            graspOrientation = [objectPose.orientation.x, objectPose.orientation.y,
                                objectPose.orientation.z, objectPose.orientation.w]
            graspOrientation = quaternion_multiply(
                graspOrientation, quaternion_from_euler(0, 0, math.pi/2 * (corner + 1.0/2.0)))
            for rotationNumber in range(4):
                graspPose.position = copy.deepcopy(objectPose.position)
                q = quaternion_multiply(graspOrientation, quaternion_from_euler(
                    rotationNumber * math.pi / 2, 0, 0))
                graspPose.orientation.x = q[0]
                graspPose.orientation.y = q[1]
                graspPose.orientation.z = q[2]
                graspPose.orientation.w = q[3]
                self.addGrasp(baseLink, graspPose)

        # select corner connected to x and z faces
        for corner in range(4):
            graspOrientation = [objectPose.orientation.x, objectPose.orientation.y,
                                objectPose.orientation.z, objectPose.orientation.w]
            graspOrientation = quaternion_multiply(graspOrientation, quaternion_from_euler(math.pi/2, 0, 0))
            graspOrientation = quaternion_multiply(
                graspOrientation, quaternion_from_euler(0, 0, math.pi/2 * (corner + 1.0/2.0)))
            for rotationNumber in range(4):
                graspPose.position = copy.deepcopy(objectPose.position)
                q = quaternion_multiply(graspOrientation, quaternion_from_euler(
                    rotationNumber * math.pi / 2, 0, 0))
                graspPose.orientation.x = q[0]
                graspPose.orientation.y = q[1]
                graspPose.orientation.z = q[2]
                graspPose.orientation.w = q[3]
                self.addGrasp(baseLink, graspPose)

        # select corner connected to y and z faces
        for corner in range(4):
            graspOrientation = [objectPose.orientation.x, objectPose.orientation.y,
                                objectPose.orientation.z, objectPose.orientation.w]
            # graspOrientation = quaternion_multiply(graspOrientation, quaternion_from_euler(math.pi/2, 0, 0))
            graspOrientation = quaternion_multiply(graspOrientation, quaternion_from_euler(0, math.pi/2, 0))
            graspOrientation = quaternion_multiply(
                graspOrientation, quaternion_from_euler(0, 0, math.pi/2 * (corner + 1.0/2.0)))
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
        """[summary]

        :param baseLink: Base link name
        :type baseLink: String
        :param graspPose: pose of the grasp to add
        :type graspPose: geometry_msgs/Pose
        """
        grasp = Grasp()

        grasp.id = str(len(self.grasps))
        grasp.grasp_pose.header.frame_id = baseLink
        grasp.grasp_pose.pose.position.x = graspPose.position.x + self._eef_to_transform[0]
        grasp.grasp_pose.pose.position.y = graspPose.position.y + self._eef_to_transform[1]
        grasp.grasp_pose.pose.position.z = graspPose.position.z + self._eef_to_transform[2]
        eefRotation = quaternion_from_euler(
            self._eef_to_transform[3], self._eef_to_transform[4], self._eef_to_transform[5])
        graspRotation = [graspPose.orientation.x, graspPose.orientation.y,
                         graspPose.orientation.z, graspPose.orientation.w]
        finalOrientation = quaternion_multiply(graspRotation, eefRotation)
        grasp.grasp_pose.pose.orientation.x = finalOrientation[0]
        grasp.grasp_pose.pose.orientation.y = finalOrientation[1]
        grasp.grasp_pose.pose.orientation.z = finalOrientation[2]
        grasp.grasp_pose.pose.orientation.w = finalOrientation[3]

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

    def setParameter(self, graspXFace, graspYFace, graspZFace, graspEdge, planCorner):
        """Set grasp generation possibility

        :param graspXFace: Generate face oriented grasps on X and -X faces
        :type graspXFace: Bool
        :param graspYFace: Generate face oriented grasps on Y and -Y faces
        :type graspYFace: Bool
        :param graspZFace: Generate face oriented grasps on Z and -Z faces
        :type graspZFace: Bool
        :param graspEdge: Generate edge grasping
        :type graspEdge: Bool
        :param planCorner: Generate corner grasping
        :type planCorner: Bool
        """
        self._graspXFace = graspXFace
        self._graspYFace = graspYFace
        self._graspZFace = graspZFace
        self._graspEdge = graspEdge
        self._planCorner = planCorner