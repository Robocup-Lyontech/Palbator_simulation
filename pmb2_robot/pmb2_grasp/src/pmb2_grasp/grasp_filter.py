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


        self._collisionCheck = True
        self._preGraspCheck = True
        self._planCheck = True

    def filterGrasps(self, grasp_candidates, objectName, graspParam):
        rospy.loginfo("{class_name} : Filtering grasps".format(class_name=self.__class__.__name__))
        self.filteredGrasps = copy.deepcopy(grasp_candidates)

        if self._collisionCheck:
            nbrRemoved = self.removeCollision(graspParam)
            rospy.loginfo("{class_name} : Grasp filtered by collision: %d".format(class_name=self.__class__.__name__), nbrRemoved)

        if self._preGraspCheck:
            nbrRemoved = self.removePreGrasp(graspParam)
            rospy.loginfo("{class_name} : Grasp filtered by pre-grasp: %d".format(class_name=self.__class__.__name__), nbrRemoved)

        if self._planCheck:
            nbrRemoved = self.removePlan(objectName, graspParam)
            rospy.loginfo("{class_name} : Grasp filtered by plan: %d".format(class_name=self.__class__.__name__), nbrRemoved)
        
        rospy.loginfo("{class_name} : Grasps left after filter: %d".format(class_name=self.__class__.__name__), len(self.filteredGrasps))
        return self.filteredGrasps

    def removeGraspsWithId(self, ids):
        to_remove = [i for i, val in enumerate(self.filteredGrasps) if val.id in ids]
        for index in reversed(to_remove):
            del self.filteredGrasps[index]

    def removeCollision(self, graspParam):
        idsToRemove = []
        for grasp_candidate in self.filteredGrasps:
            if not self.filterCollision(grasp_candidate, graspParam):
                idsToRemove.append(grasp_candidate.id)
        self.removeGraspsWithId(idsToRemove)
        return len(idsToRemove)
        

    def filterCollision(self, grasp_candidate, graspParam):
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

    def removePreGrasp(self, graspParam):
        idsToRemove = []
        for grasp_candidate in self.filteredGrasps:
            if not self.filterPreGrasp(grasp_candidate, graspParam):
                idsToRemove.append(grasp_candidate.id)
        self.removeGraspsWithId(idsToRemove)
        return len(idsToRemove)

    def filterPreGrasp(self, grasp_candidate, graspParam):
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