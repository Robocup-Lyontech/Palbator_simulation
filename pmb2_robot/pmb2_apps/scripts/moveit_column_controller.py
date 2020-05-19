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


class MoveitColumnController:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("pmb2_column")

        self.display_trajectory_publisher = rospy.Publisher('/move_group_column/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=10)
        self.minimum_column = 0.35 + 0.06 #offset for gazebo
        self.maximum_column = 1.35
        rospy.logwarn("COLUMN CONTROLLER ON")

    # def move_column_sub(self,req):

    #     column_pose_target = Pose()

    #     column_pose_target.position.z = req.data
    #     column_pose_target.orientation.w = 1.0
    #     self.group.set_pose_reference_frame("base_footprint")
    #     self.group.set_joint_value_target(column_pose_target,True)

    #     plan1 = self.group.plan()
    #     self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    #     self.display_trajectory.trajectory_start = self.robot.get_current_state()
    #     self.display_trajectory.trajectory.append(plan1)
    #     self.display_trajectory_publisher.publish(self.display_trajectory)
    #     print "============ Waiting while RVIZ displays plan1..."
    #     rospy.sleep(5)
    #     self.group.go(wait=True)

    def move_column(self,z_target):
        column_pose_target = Pose()
        if z_target < self.minimum_column:
            column_pose_target.position.z = self.minimum_column

        elif z_target > self.maximum_column:
            column_pose_target.position.z = self.maximum_column

        else:
            column_pose_target.position.z = z_target
        

        column_pose_target.orientation.w = 1.0
        self.group.set_pose_reference_frame("base_footprint")
        self.group.set_joint_value_target(column_pose_target,True)

        plan1 = self.group.plan()
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan1)
        self.display_trajectory_publisher.publish(self.display_trajectory)
        print "============ Waiting while RVIZ displays move_column_plan..."
        rospy.sleep(5)
        self.group.go(wait=True)


if __name__ == "__main__":

    column_controller = MoveitColumnController()
    while not rospy.is_shutdown():
        rospy.spin()


    