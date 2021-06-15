#!/usr/bin/env python
import rospy
import actionlib
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotState, PositionIKRequest, PickupAction, PickupGoal, PlanningOptions
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
import copy


class GraspFilter:

    def __init__(self):
        self.IKSolver = rospy.ServiceProxy("compute_ik", GetPositionIK)
        #TODO add wait for service

        self.pickPlan = actionlib.SimpleActionClient("pickup", PickupAction)
        #TODO add wait for action


        self._IKGraspCheck = True       # Filter grasp by IK
        self._IKPreGraspCheck = True    # Filter pre-grasp by IK
        self._planCheck = True          # Filter full plan feasibility

    def filterGrasps(self, grasp_candidates, objectName, graspParam):
        """Check if grasp feasibility and return possible one

        :param grasp_candidates: List of grasp candidate to filter
        :type grasp_candidates: moveit_msgs/Grasp[]
        :param objectName: Name of the object to grasp
        :type objectName: String
        :param graspParam: Parameter of the grasp
        :type graspParam: Dict
        :return: List of grasp after filter
        :rtype: moveit_msgs/Grasp[]
        """
        rospy.loginfo("{class_name} : Filtering grasps".format(class_name=self.__class__.__name__))
        self.filteredGrasps = copy.deepcopy(grasp_candidates)

        if self._IKGraspCheck:
            nbrRemoved = self.removeIKGrasp(graspParam)
            rospy.loginfo("{class_name} : Grasp filtered by ik: %d".format(class_name=self.__class__.__name__), nbrRemoved)

        if self._IKPreGraspCheck:
            nbrRemoved = self.removeIKPreGrasp(graspParam)
            rospy.loginfo("{class_name} : Grasp filtered by ik pre-grasp: %d".format(class_name=self.__class__.__name__), nbrRemoved)

        if self._planCheck:
            nbrRemoved = self.removePlan(objectName, graspParam)
            rospy.loginfo("{class_name} : Grasp filtered by plan: %d".format(class_name=self.__class__.__name__), nbrRemoved)
        
        rospy.loginfo("{class_name} : Grasps left after filter: %d".format(class_name=self.__class__.__name__), len(self.filteredGrasps))
        return self.filteredGrasps

    def removeGraspsWithId(self, ids):
        """Remove grasp with ids matching from filtered grasp list

        :param ids: List of id to remove
        :type ids: Int[]
        """
        to_remove = [i for i, val in enumerate(self.filteredGrasps) if val.id in ids]
        for index in reversed(to_remove):
            del self.filteredGrasps[index]

    def removeIKGrasp(self, graspParam):
        """Remove grasp where no IK were found during grasp step

        :param graspParam: Parameter of the grasp
        :type graspParam: Dict
        :return: Number of grasp removed
        :rtype: Int
        """
        idsToRemove = []
        for grasp_candidate in self.filteredGrasps:
            if not self.filterIKGrasp(grasp_candidate, graspParam):
                idsToRemove.append(grasp_candidate.id)
        self.removeGraspsWithId(idsToRemove)
        return len(idsToRemove)
        

    def filterIKGrasp(self, grasp_candidate, graspParam):
        positionIKRequest = PositionIKRequest()
        positionIKRequest.group_name = graspParam["arm_group"]
        joint_state = JointState()
        joint_state.name = graspParam["joints"]
        joint_state.position = graspParam["pregrasp_posture"]
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        positionIKRequest.robot_state = moveit_robot_state
        positionIKRequest.pose_stamped = copy.deepcopy(grasp_candidate.grasp_pose)
        positionIKRequest.timeout = rospy.Duration(graspParam["IK_solver_timeout"])
        positionIKRequest.attempts = graspParam["IK_solver_attempts"]
        positionIKRequest.avoid_collisions = True

        resp = self.IKSolver.call(positionIKRequest)
        if resp.error_code.val == 1:
            return True
        return False

    def removeIKPreGrasp(self, graspParam):
        idsToRemove = []
        for grasp_candidate in self.filteredGrasps:
            if not self.filterIKPreGrasp(grasp_candidate, graspParam):
                idsToRemove.append(grasp_candidate.id)
        self.removeGraspsWithId(idsToRemove)
        return len(idsToRemove)

    def filterIKPreGrasp(self, grasp_candidate, graspParam):
        positionIKRequest = PositionIKRequest()
        positionIKRequest.group_name = graspParam["arm_group"]
        joint_state = JointState()
        joint_state.name = graspParam["joints"]
        joint_state.position = graspParam["pregrasp_posture"]
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        positionIKRequest.robot_state = moveit_robot_state
        positionIKRequest.pose_stamped = copy.deepcopy(grasp_candidate.grasp_pose)
        positionIKRequest.pose_stamped.pose.position.x -= graspParam["eef_approach_vector"][0] * graspParam["min_approach_distance"]
        positionIKRequest.pose_stamped.pose.position.y -= graspParam["eef_approach_vector"][1] * graspParam["min_approach_distance"]
        positionIKRequest.pose_stamped.pose.position.z -= graspParam["eef_approach_vector"][2] * graspParam["min_approach_distance"]
        positionIKRequest.timeout = rospy.Duration(graspParam["IK_solver_timeout"])
        positionIKRequest.attempts = graspParam["IK_solver_attempts"]
        positionIKRequest.avoid_collisions = True

        resp = self.IKSolver.call(positionIKRequest)
        if resp.error_code.val == 1:
            return True
        return False

    def removePlan(self, objectName, graspParam):
        idsToRemove = []
        for grasp_candidate in self.filteredGrasps:
            if not self.filterPlan(objectName, grasp_candidate, graspParam):
                idsToRemove.append(grasp_candidate.id)
        self.removeGraspsWithId(idsToRemove)
        return len(idsToRemove)

    def filterPlan(self, objectName, grasp_candidate, graspParam):
        pickup = PickupGoal()
        pickup.target_name = objectName
        pickup.group_name = graspParam["arm_group"]
        pickup.end_effector = graspParam["eef_group"]
        pickup.possible_grasps.append(grasp_candidate)
        pickup.allowed_planning_time = graspParam["full_plan_timeout"]
        pickup.planning_options = PlanningOptions()
        pickup.planning_options.plan_only = True

        self.pickPlan.send_goal(pickup)
        self.pickPlan.wait_for_result()
        resp = self.pickPlan.get_result()

        if resp.error_code.val == 1:
            return True
        return False

    def setParameter(self, IKGraspCheck, IKPreGraspCheck, planCheck):
        """Set filter to use

        :param IKGraspCheck: Check collision on grasp position
        :type IKGraspCheck: Bool
        :param IKPreGraspCheck: Check collision in pre grasp position (target object included)
        :type IKPreGraspCheck: Bool
        :param planCheck: Check all path
        :type planCheck: Bool
        """
        self._IKGraspCheck = IKGraspCheck
        self._IKPreGraspCheck = IKPreGraspCheck
        self._planCheck = planCheck