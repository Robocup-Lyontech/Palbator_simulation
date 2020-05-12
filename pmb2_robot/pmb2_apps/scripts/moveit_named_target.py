#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg



class MoveitArmControl:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("pmb2_arm")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,queue_size=10)
        print "============ Waiting for RVIZ..."
        rospy.sleep(5)
        print "============ Starting tutorial "

    def move_to_poses(self,pose_name):
        self.group.set_named_target(pose_name)

        plan1 = self.group.plan()
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan1)
        self.display_trajectory_publisher.publish(self.display_trajectory)
        print "============ Waiting while RVIZ displays plan1..."
        # rospy.sleep(5)
        self.group.go(wait=True)

    def change_target_name(self, nameTarget):
        if nameTarget == "B":
            nameTarget = "D"
        elif nameTarget == "D":
            nameTarget = "B"
        return nameTarget

if __name__ == '__main__':

    pose_name="travelling_pose"
    a = MoveitArmControl()
    a.move_to_poses(pose_name)
