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

class MoveitArmControl:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        # self.group = moveit_commander.MoveGroupCommander("pmb2_arm_without_column")
        self.group = moveit_commander.MoveGroupCommander("pmb2_arm")

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,queue_size=10)
        
        
        print "============ Waiting for RVIZ..."
        rospy.sleep(3)
        print "============ Starting tutorial "

        self.setup_box()
        # now = rospy.Time(0)
        # self._tflistener = TransformListener()
        # self._tflistener.waitForTransform("base_footprint", "palbator_arm_end_link", now, rospy.Duration(2))
        # (trans, rot) = self._tflistener.lookupTransform("base_footprint", "palbator_arm_end_link", now)

        # rospy.logwarn(trans)
        # rospy.logwarn(rot)

        self.sub = rospy.Subscriber("/test_pose",Pose,self.setup_point)

        self.sub2 = rospy.Subscriber("/test_angle",Float32,self.setup_angle)

        self.sub3 = rospy.Subscriber("/move_to_name_pose",String,self.move_to_poses)

    def setup_angle(self,req):

        lg_bras = 0.782

        angle = req.data

        
        pose_target = Pose()



        pose_target.position.z = 1.29

        # self._tflistener.waitForTransform("/map", "base_footprint", now, rospy.Duration(2))
        # (trans, rot) = self._tflistener.lookupTransform("/map", "base_footprint", now)
        # robotPose = Pose()
        # robotPose.position.x = trans[0]
        # robotPose.position.y = trans[1]
        # robotPose.position.z = trans[2]

        # quaternion = (rot[0], rot[1], rot[2], rot[3])
        # euler = tf.transformations.euler_from_quaternion(quaternion)
        # roll = euler[0]
        # pitch = euler[1]
        # yaw = euler[2] + data
        # q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        # robotPose.orientation.x = q[0]
        # robotPose.orientation.y = q[1]
        # robotPose.orientation.z = q[2]
        # robotPose.orientation.w = q[3]

        
        # x: 0.45
        # y: -0.6
        # z: 1.29

    def setup_point(self,req):

        position_effector=self.group.get_current_pose(self.group.get_end_effector_link())
        rospy.loginfo(position_effector)
        pose_target = Pose()
        pose_target.position = req.position
        pose_target.orientation = req.orientation
        self.group.set_pose_reference_frame("base_footprint")
        # self.group.clear_pose_targets()

        # waypoints = []
        # pose_target.position.x = -0.5
        # pose_target.position.y = 0.48
        # pose_target.position.z = 0.0
        # pose_target.orientation.x = 0
        # pose_target.orientation.y = 0
        # pose_target.orientation.z = 0.5224
        # pose_target.orientation.w = -0.85



        # waypoints.append(copy.deepcopy(pose_target))
        # pose_target.orientation.x = 0.0
        # pose_target.orientation.x = 0.0
        # pose_target.orientation.z = 1.0
        # pose_target.orientation.w = 0.0



        rospy.logwarn(pose_target)

        # (plan3, fraction) = self.group.compute_cartesian_path(
        #     waypoints,  # waypoints to follow
        #     0.01,  # eef_step
        #     0.0)  # jump_threshold

        # self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        # # self.display_trajectory.trajectory_start = self.robot.get_current_state()
        # self.display_trajectory.trajectory.append(plan3)
        # self.display_trajectory_publisher.publish(self.display_trajectory)

        # print "============ Waiting while RVIZ displays plan3..."
        # rospy.sleep(5)
        # self.group.execute(plan3,wait=True)

        self.group.set_joint_value_target(pose_target,True)

        plan1 = self.group.plan()
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan1)
        self.display_trajectory_publisher.publish(self.display_trajectory)
        print "============ Waiting while RVIZ displays plan1..."
        rospy.sleep(5)
        self.group.go(wait=True)

        

    def setup_box(self):

        box_pose = geometry_msgs.msg.PoseStamped()

        box_name = "table1"
        box_pose.header.frame_id = "base_link"

        box_pose.pose.position.x = 2.0
        box_pose.pose.position.y = -1.0
        box_pose.pose.position.z = 0.25
        box_pose.pose.orientation.w = 1.0

        box_size = (0.5, 0.5, 0.5)
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

    def move_to_poses(self,req):

        pose_name = req.data

        self.group.set_named_target(pose_name)

        plan1 = self.group.plan()
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan1)
        self.display_trajectory_publisher.publish(self.display_trajectory)
        print "============ Waiting while RVIZ displays plan1..."
        # rospy.sleep(5)
        self.group.go(wait=True)


if __name__ == '__main__':

    pose_name="travelling_pose"
    a = MoveitArmControl()
    # a.move_to_poses(pose_name)
    while not rospy.is_shutdown():
        rospy.spin()
