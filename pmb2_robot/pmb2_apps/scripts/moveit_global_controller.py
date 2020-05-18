#!/usr/bin/env python
import rospy
from moveit_column_controller import MoveitColumnController
from moveit_arm_controller import MoveitArmController
from geometry_msgs.msg import Point, PoseStamped
import moveit_commander
import sys

import actionlib
from pmb2_apps.msg import ArmControlAction, ArmControlResult

from tf import TransformListener
from geometry_msgs.msg import PointStamped



class MoveitGlobalController:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_global_controller',anonymous=True)
        self.scene = moveit_commander.PlanningSceneInterface()

        self._arm_controller = MoveitArmController()
        self._column_controller = MoveitColumnController()
        self.first_move = True
        self.sub = rospy.Subscriber("/global_point",Point,self.handle_new_point)

        self._arm_control_server = actionlib.SimpleActionServer("arm_control_action",ArmControlAction,self.executeActionServer, False)
        self._arm_control_server.start()


    def executeActionServer(self,goal):
        isActionSucceed = False
        action_result = ArmControlResult()
        try:
            listener = TransformListener()
            object_name_TF = goal.object_label
            now = rospy.Time(0)
            listener.waitForTransform("/map", object_name_TF, now, rospy.Duration(2))
            (trans, rot) = listener.lookupTransform("/map", object_name_TF, now)

            object_point = PointStamped()
            object_point.header.frame_id = "map"
            object_point.header.stamp = rospy.Time(0)
            object_point.point.x = trans[0]
            object_point.point.y = trans[1]
            object_point.point.z = trans[2]

            rospy.loginfo("Object coords in map : %s",str(object_point))


            # rospy.sleep(5)

            p = listener.transformPoint("base_footprint",object_point)

            rospy.loginfo("Object coords in base_footprint : %s",str(p))

            if self.first_move == True:
                self.first_move = False
                self._arm_controller.move_arm_to_pose("pointing_pose")

            self._arm_controller.move_arm(p.point.x,p.point.y)

            # self._column_controller.move_column(req.z)

            isActionSucceed = True

        except Exception as e:
            rospy.logwarn("unable to find or launch function corresponding to the action %s:, error:[%s]",str(goal.action), str(e))

        if isActionSucceed:
            rospy.loginfo("Action %s succeeded",goal.action)
            action_result.action_output = 'OK'
            self._arm_control_server.set_succeeded(action_result)
        else:
            rospy.loginfo("Action %s aborted",goal.action)
            self._arm_control_server.set_aborted()
        return






    def handle_new_point(self,req):

        # self.setup_box(req.x,req.y,req.z)

        # rospy.sleep(3)

        object_name = "object_bleach0_TF"

        if self.first_move == True:
            self.first_move = False
            self._arm_controller.move_arm_to_pose("pointing_pose")

        self._arm_controller.move_arm(req.x,req.y)

        self._column_controller.move_column(req.z)


    def setup_box(self,x,y,z):

        box_pose = PoseStamped()

        box_name = "table1"
        box_pose.header.frame_id = "base_footprint"

        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z
        box_pose.pose.orientation.w = 1.0

        box_size = (0.25, 0.25, 0.25)
        self.box_name = box_name

        self.scene.add_box(box_name, box_pose, size=box_size)
        return self.wait_for_state_update(box_is_known=True)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

if __name__ == "__main__":

    global_controller = MoveitGlobalController()
    while not rospy.is_shutdown():
        rospy.spin()
