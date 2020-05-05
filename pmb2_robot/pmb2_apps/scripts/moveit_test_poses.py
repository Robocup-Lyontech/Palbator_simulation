#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg



class moveit_test_poses:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("pm2b_arm_group")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,queue_size=10)
        print "============ Waiting for RVIZ..."
        rospy.sleep(5)
        print "============ Starting tutorial "

    def move_to_poses(self,pose_name):
        pose_target = geometry_msgs.msg.Pose()
        self.group.set_pose_reference_frame("base_link")
        if pose_name=="B":
            pose_target.position.x = -0.143328
            pose_target.position.y = -0.350759
            pose_target.position.z = 1.0

        elif pose_name=="D":
            pose_target.position.x = -0.143328
            pose_target.position.y = -0.350759
            pose_target.position.z = 0.231302


        self.group.set_joint_value_target(pose_target,True)

        plan1 = self.group.plan()
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan1)
        self.display_trajectory_publisher.publish(self.display_trajectory)
        print "============ Waiting while RVIZ displays plan1..."
        rospy.sleep(5)
        self.group.go(wait=True)

    def change_target_name(self, nameTarget):
        if nameTarget == "B":
            nameTarget = "D"
        elif nameTarget == "D":
            nameTarget = "B"
        return nameTarget

if __name__ == '__main__':

    pose_name="B"
    a = moveit_test_poses()
    while not rospy.is_shutdown():
        a.move_to_poses(pose_name)
        pose_name=a.change_target_name(pose_name)
        print(pose_name)
        #rospy.spin()
