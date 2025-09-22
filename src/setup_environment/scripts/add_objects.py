#!/usr/bin/env python3
import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def add_table():
    rospy.init_node("add_table_node", anonymous=True)
    scene = PlanningSceneInterface()
    rospy.sleep(2)

    table_pose = PoseStamped()
    table_pose.header.frame_id = "base_link"
    table_pose.pose.position.x = 0.8
    table_pose.pose.position.y = 0.0
    table_pose.pose.position.z = 0.4
    table_pose.pose.orientation.w = 1.0

    scene.add_box("table", table_pose, size=(1.2, 0.6, 0.8))
    rospy.loginfo("Đã thêm bàn vào môi trường MoveIt.")

if __name__ == "__main__":
    add_table()
