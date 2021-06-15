#!/usr/bin/env python
import traceback
import rospy
from moveit_column_controller import MoveitColumnController
from moveit_arm_controller import MoveitArmController
from moveit_gripper_controller import MoveitGripperController
from geometry_msgs.msg import PoseStamped
import moveit_commander
import sys

import actionlib
from pmb2_apps.msg import ArmControlAction, ArmControlResult

from tf import TransformListener
from std_msgs.msg import Float64
import json
import math
from copy import deepcopy


class MoveitGlobalController:

    def __init__(self):
        """
        Initializes the global controller which will control Palbator Moveit arm and column controllers
        """
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_global_controller', anonymous=True)
        self.scene = moveit_commander.PlanningSceneInterface()

        if rospy.has_param("~Palbator_column_parameters"):
            column_parameters = rospy.get_param("~Palbator_column_parameters")
            self._column_controller = MoveitColumnController(column_parameters)
        else:
            rospy.logerr("{class_name} : No parameters specified for Moveit Palbator Column controller. Can't start column controller.".format(
                class_name=self.__class__.__name__))

        if rospy.has_param("~Palbator_arm_parameters"):
            arm_parameters = rospy.get_param("~Palbator_arm_parameters")
            self._arm_controller = MoveitArmController(arm_parameters)
        else:
            rospy.logerr("{class_name} : No parameters specified for Moveit Palbator Arm controller. Can't start arm controller.".format(
                class_name=self.__class__.__name__))

        if rospy.has_param("~Palbator_gripper_parameters"):
            gripper_parameters = rospy.get_param("~Palbator_gripper_parameters")
            self._gripper_controller = MoveitGripperController(gripper_parameters)
        else:
            rospy.logerr("{class_name} : No parameters specified for Moveit Palbator Gripper controller. Can't start gripper controller.".format(
                class_name=self.__class__.__name__))

        if rospy.has_param("~" + rospy.get_param("~grasp_group_name")):
            self.grasp_parameters=rospy.get_param("~" + rospy.get_param("~grasp_group_name"))
        else:
            rospy.logerr("{class_name} : No parameters specified for grasp.".format(
                class_name=self.__class__.__name__))

        self.__setFingertip()
        self.traveling()

        if rospy.has_param("~Moveit_global_controller_action_name"):
            action_server_name=rospy.get_param("~Moveit_global_controller_action_name")
            self._arm_control_server=actionlib.SimpleActionServer(
                action_server_name, ArmControlAction, self.executeActionServer, False)
            self._arm_control_server.start()
        else:
            rospy.logerr(
                "{class_name} : No name specified for Global Moveit Palbator controller action server. Can't start action server.")

        rospy.loginfo("{class_name} : Global Palbator Moveit Controller initialized".format(
            class_name=self.__class__.__name__))

    def __setFingertip(self):
        pub=rospy.Publisher("/pmb2_arm_controller/arm_hand1_fingertip1_joint_controller/command",
                              Float64, queue_size=10)
        pub.publish(0.0)
        pub=rospy.Publisher("/pmb2_arm_controller/arm_hand1_fingertip2_joint_controller/command",
                              Float64, queue_size=10)
        pub.publish(0.0)
        pub=rospy.Publisher("/pmb2_arm_controller/arm_hand2_fingertip1_joint_controller/command",
                              Float64, queue_size=10)
        pub.publish(0.0)
        pub=rospy.Publisher("/pmb2_arm_controller/arm_hand2_fingertip2_joint_controller/command",
                              Float64, queue_size=10)
        pub.publish(0.0)

    def __getPoseStamped(self, goal):
        """
        Transform from ArmControlGoal type to PoseStamped transformed to base_footptrint

        :param goal: goal with object label or xyz coord
        :type goal: ArmControlGoal
        :raises Exception: can't extract PoseStamped
        :return: Posestamped of the goal in base_footprint frame
        :rtype: PoseStamped
        """

        now=rospy.Time(0)

        try:
            listener=TransformListener()

            if goal.action.endswith("XYZ"):
                poseStamped=PoseStamped()
                poseStamped.header.frame_id="map"
                poseStamped.header.stamp=now
                poseStamped.pose=deepcopy(goal.pose)

            else:
                listener.waitForTransform("map", goal.object_label, now, rospy.Duration(5))
                (trans, rot)=listener.lookupTransform("map", goal.object_label, now)

                poseStamped=PoseStamped()
                poseStamped.header.frame_id="map"
                poseStamped.header.stamp=now
                poseStamped.pose.position.x=trans[0]
                poseStamped.pose.position.y=trans[1]
                poseStamped.pose.position.z=trans[2]
                poseStamped.pose.orientation.x=rot[0]
                poseStamped.pose.orientation.y=rot[1]
                poseStamped.pose.orientation.z=rot[2]
                poseStamped.pose.orientation.w=rot[3]

            rospy.loginfo("{class_name} : Goal coords in map : %s".format(
                class_name=self.__class__.__name__), str(poseStamped.pose))
            listener.waitForTransform("/map", "/base_footprint", now, rospy.Duration(5))
            poseStampedTransformed=listener.transformPose("base_footprint", poseStamped)

        except Exception as exc:
            raise Exception("{class_name} : Error getting coord of the goal: %s".format(
                class_name=self.__class__.__name__), exc)

        return poseStampedTransformed

    def __getPlacement(self, poseStamped, action):
        """
        Function used to detect if the movement is possible and return the movement to do if not

        :param poseStamped: coord of the goal
        :type poseStamped: PoseStamped
        :param action: action asked to do
        :type action: String
        :return: rotation needed in radian and distance needed to grab (negativ mean too close)
        :rtype: Float, Float
        """
        # angle calculation
        alpha=math.atan2(poseStamped.pose.position.y, poseStamped.pose.position.x)
        rospy.logwarn("{class_name} : ANGLE TO GOAL %.2f Radian %d Degree".format(
            class_name=self.__class__.__name__), alpha, alpha*180/math.pi)

        if (poseStamped.pose.position.x > 0):
            if (poseStamped.pose.position.y > 0):
                rotationNeeded=-math.pi/2
            else:
                rotationNeeded=0.0
        else:
            if (poseStamped.pose.position.y > 0):
                rotationNeeded=-math.pi
            else:
                rotationNeeded=math.pi/2

        # distance calculation
        distance=math.sqrt(poseStamped.pose.position.x**2 + poseStamped.pose.position.y**2)
        rospy.logwarn("{class_name} : DISTANCE TO GOAL %.3f Meter".format(
            class_name=self.__class__.__name__), distance)

        if distance >= self._arm_controller.arm_max_length or distance <= self._arm_controller.arm_min_length:
            distanceNeeded=distance - self._arm_controller.arm_max_length
        else:
            distanceNeeded=0

        return rotationNeeded, distanceNeeded

    def grasping(self, goal):
        # put the arm in base pose before grab
        # self._arm_controller.move_arm_to_pose("pointing_pose")
        lookingGoal = deepcopy(goal)
        lookingGoal.pose.position.z += 0.1
        self.looking(lookingGoal)

        # align end effector and object to grab
        self._column_controller.move_column(goal.pose.position.z + 0.1)

        # move end effector to the object
        self._arm_controller.grab(goal, self.solidPrimitive, self.grasp_parameters)

        self._arm_controller.move_arm_to_pose("pointing_pose")
        # self._arm_controller.post_grab(goal)

    def pointing(self, goal):
        self._arm_controller.point_at(goal)
        self._column_controller.move_column(goal.pose.position.z)

    def dropping(self, goal):
        # align end effector and position to drop
        self._arm_controller.point_at(goal)
        self._column_controller.move_column(goal.pose.position.z)

        # move end effector to the position to drop
        self._arm_controller.drop(goal)
        # drop
        self._gripper_controller.move_gripper(1.0)
        # move arm to standby
        self._arm_controller.move_arm_to_pose("pointing_pose")

    def looking(self, goal):
        # set initial pose
        self._arm_controller.move_arm_to_pose("looking_pose")

        # align camera with goal
        zGoal = max(goal.pose.position.z, 0.62)
        self._column_controller.move_column(zGoal)
        self._arm_controller.look_at(goal)

    def traveling(self):
        self._column_controller.move_column_to_pose("travelling_pose")
        self._arm_controller.move_arm_to_pose("travelling_pose")
        self._gripper_controller.move_gripper_to_pose("close_gripper")

    def executeActionServer(self, goal):
        """
        Action Server callback for Moveit global control. Can point an object or move in a defined position to travel without risks.
        :param goal: contains action data to do
        :type goal: ArmControlGoal
        """
        isActionSucceed=False
        JSONRequest={}
        action_result=ArmControlResult()
        try:
            if goal.action in ["Grasping", "GraspingXYZ", "Pointing", "PointingXYZ", "Dropping", "DroppingXYZ", "Looking", "LookingXYZ"]:
                rospy.loginfo("{class_name} : Received %s action goal".format(
                    class_name=self.__class__.__name__), goal.action)

                goalPoseStamped=self.__getPoseStamped(goal)
                self.solidPrimitive = goal.solidPrimitive
                rospy.loginfo("{class_name} : Goal coords in base_footprint : %s".format(
                    class_name=self.__class__.__name__), goalPoseStamped)

                (rotation_needed, distance_needed)=self.__getPlacement(goalPoseStamped, goal.action)
                fct=None

                if "Grasping" in goal.action:
                    if rotation_needed != 0:
                        rospy.logwarn("{class_name} : ROTATION OF %.2f RADIAN NEEDED".format(
                            class_name=self.__class__.__name__), rotation_needed)
                        JSONRequest["rotation"]=rotation_needed

                    if distance_needed != 0:
                        rospy.logwarn("{class_name} : DISTANCE OF %.3f METER NEEDED".format(
                            class_name=self.__class__.__name__), distance_needed)
                        JSONRequest["distance"]=distance_needed
                    fct=self.grasping

                elif "Pointing" in goal.action:
                    if rotation_needed != 0:
                        rospy.logwarn("{class_name} : ROTATION OF %.2f RADIAN NEEDED".format(
                            class_name=self.__class__.__name__), rotation_needed)
                        JSONRequest["rotation"]=rotation_needed
                    fct=self.pointing

                elif "Dropping" in goal.action:
                    if rotation_needed != 0:
                        rospy.logwarn("{class_name} : ROTATION OF %.2f RADIAN NEEDED".format(
                            class_name=self.__class__.__name__), rotation_needed)
                        JSONRequest["rotation"]=rotation_needed

                    if distance_needed != 0:
                        rospy.logwarn("{class_name} : DISTANCE OF %.3f METER NEEDED".format(
                            class_name=self.__class__.__name__), distance_needed)
                        JSONRequest["distance"]=distance_needed
                    fct=self.dropping

                elif "Looking" in goal.action:
                    if rotation_needed != 0:
                        rospy.logwarn("{class_name} : ROTATION OF %.2f RADIAN NEEDED".format(
                            class_name=self.__class__.__name__), rotation_needed)
                        JSONRequest["rotation"]=rotation_needed
                    fct=self.looking

                if len(JSONRequest) == 0:
                    fct(goalPoseStamped)
                    isActionSucceed=True

            elif goal.action == 'Travelling':
                rospy.loginfo("{class_name} : Received travelling action goal".format(
                    class_name=self.__class__.__name__))
                self.traveling()
                isActionSucceed=True

            else:
                rospy.logwarn("{class_name} : unable to find or launch function corresponding to the action %s".format(
                    class_name=self.__class__.__name__), str(goal.action))

            json_result={
                "action": goal.action,
                "request": JSONRequest,
                "status": ('Success' if isActionSucceed else 'Aborted')
            }
            action_result.action_output=json.dumps(json_result)
            rospy.loginfo("{class_name} : Action %s %s".format(class_name=self.__class__.__name__),
                          goal.action, ('Succeded' if isActionSucceed else 'Aborted'))
            if isActionSucceed:
                self._arm_control_server.set_succeeded(action_result)
            else:
                self._arm_control_server.set_aborted(action_result)

        except Exception as exc:
            rospy.logerr("{class_name} : Action %s aborted because of: %s".format(
                class_name=self.__class__.__name__), goal.action, exc)
            rospy.logerr(traceback.format_exc())
            json_result={
                "action": goal.action,
                "request": "",
                "status": 'Aborted'
            }
            action_result.action_output=json.dumps(json_result)
            self._arm_control_server.set_aborted(action_result)


if __name__ == "__main__":

    global_controller=MoveitGlobalController()
    rospy.spin()
