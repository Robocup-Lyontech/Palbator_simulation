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
import math
import numpy as np

class MoveitArmController:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("pmb2_arm")

        self.display_trajectory_publisher = rospy.Publisher('/move_group_arm/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=10)
        self._tflistener = TransformListener()
        self.arm_length = 0.608
        

        rospy.logwarn("ARM CONTROLLER ON")

    def move_arm_to_pose(self,pose_name):
        
        self.group.set_named_target(pose_name)

        plan1 = self.group.plan()
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan1)
        self.display_trajectory_publisher.publish(self.display_trajectory)
        print "============ Waiting while RVIZ displays move_arm_to_pose_plan..."
        # rospy.sleep(5)
        self.group.go(wait=True)

    # def move_arm_subscriber(self,req):
    #     pose_target = Pose()
    #     pose_target.position = req.position
    #     pose_target.orientation = req.orientation

    #     position_effector=self.group.get_current_pose(self.group.get_end_effector_link())
        
    #     pose_target.position.z = position_effector.pose.position.z



    #     self.group.set_pose_reference_frame("base_footprint")
    #     self.group.set_joint_value_target(pose_target,True)

    #     plan1 = self.group.plan()
    #     self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    #     self.display_trajectory.trajectory_start = self.robot.get_current_state()
    #     self.display_trajectory.trajectory.append(plan1)
    #     self.display_trajectory_publisher.publish(self.display_trajectory)
    #     print "============ Waiting while RVIZ displays move_arm_plan..."
    #     rospy.sleep(5)
    #     self.group.go(wait=True)

    def move_arm(self,x_target,y_target):


        pose_target = self.calculate_pose(x_target,y_target)

        self.group.set_pose_reference_frame("base_footprint")
        self.group.set_joint_value_target(pose_target,True)

        plan1 = self.group.plan()
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan1)
        self.display_trajectory_publisher.publish(self.display_trajectory)
        print "============ Waiting while RVIZ displays plan1..."
        rospy.sleep(5)
        self.group.go(wait=True)

    def calculate_pose(self,x_target,y_target):

        arm_pose = Pose()

        alpha = np.arctan(y_target/x_target) 

        # rotation_need = False

        # if alpha < 0 and alpha > -math.pi/2:
        #     rospy.logwarn("NO ROTATION NEEDED")
            
        # elif alpha > 0 and alpha < math.pi/2:
        #     rospy.logwarn("PI/2 ROTATION NEEDED")
        #     rotation_need = True

        # elif alpha > -math.pi and alpha < -math.pi/2:
        #     rospy.logwarn("-PI/2 ROTATION NEEDED")
        #     rotation_need = True

        # if rotation_need:
        #     now = rospy.Time(0)
        #     self._tflistener.waitForTransform("/map", object_label, now, rospy.Duration(2))
        #     (trans, rot) = self._tflistener.lookupTransform("/map", object_label, now)
        #     rospy.logwarn((trans, rot))
        #     x_target = trans[0]
        #     y_target = trans[1]
        #     alpha = np.arctan(y_target/x_target)


        q = tf.transformations.quaternion_from_euler(0.0, 0.0, alpha)

        x_arm = np.cos(alpha)*self.arm_length
        y_arm = np.sin(alpha)*self.arm_length

        arm_pose.position.x = x_arm
        arm_pose.position.y = y_arm
        position_effector=self.group.get_current_pose(self.group.get_end_effector_link())
        arm_pose.position.z = position_effector.pose.position.z

        arm_pose.orientation.x = q[0]
        arm_pose.orientation.y = q[1]
        arm_pose.orientation.z = q[2]
        arm_pose.orientation.w = q[3]

        return arm_pose


# if __name__ == "__main__":
#     arm_controller = MoveitArmController()
#     while not rospy.is_shutdown():
#         rospy.spin()