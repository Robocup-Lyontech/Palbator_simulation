#!/usr/bin/env python
# 1.2 2.1
import rospy
import sys
import moveit_commander
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node("add_floors")
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 2.4
    pose.pose.position.y = 1.8
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0
    scene.add_box("wrc_ground_plane", pose, size=(10, 10, 0.0))
    rospy.sleep(1.0)

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 0.05
    pose.pose.position.y = 1.85
    pose.pose.position.z = 0.59
    pose.pose.orientation.w = 1.0
    scene.add_box("wrc_tall_table", pose, size=(0.4, 0.4, 0.02))
    rospy.sleep(1.0)

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 1.03
    pose.pose.position.y = 1.84
    pose.pose.position.z = 0.39
    pose.pose.orientation.w = 1.0
    scene.add_box("wrc_long_table", pose, size=(1.2, 0.4, 0.02))
    rospy.sleep(1.0)

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 1.52
    pose.pose.position.y = -0.58
    pose.pose.position.z = 0.39
    pose.pose.orientation.w = 1.0
    scene.add_box("wrc_long_table_0", pose, size=(1.2, 0.4, 0.02))
    rospy.sleep(1.0)

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 2.25
    pose.pose.position.y = 4.79
    pose.pose.position.z = 0.49
    pose.pose.orientation.w = 1.0
    scene.add_box("slit1", pose, size=(0.8, 0.28, 0.02))
    rospy.sleep(1.0)

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 2.25
    pose.pose.position.y = 4.79
    pose.pose.position.z = 0.79
    pose.pose.orientation.w = 1.0
    scene.add_box("slit2", pose, size=(0.8, 0.28, 0.02))
    rospy.sleep(1.0)

    rospy.loginfo("{class_name} : End of spawn".format(class_name="add_floors"))


if __name__ == "__main__":
    main()