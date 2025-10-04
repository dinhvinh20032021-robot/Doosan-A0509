#!/usr/bin/env python3

import os
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from moveit_msgs.msg import PlanningScene, ObjectColor
from rospkg import RosPack

def main():
    # Khởi ROS & MoveIt
    moveit_commander.roscpp_initialize([])
    rospy.init_node("publish_table", anonymous=True)

    # PlanningSceneInterface để add mesh
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    rospy.sleep(1)

    # Pose của bàn (-0.4, 2.20, 0.04)
    table_pose = PoseStamped()
    table_pose.header.frame_id = "world"
    table_pose.pose.position.x = -0.4
    table_pose.pose.position.y = 2.20
    table_pose.pose.position.z = 0.04

    # Scale mesh
    mesh_scale = (0.2, 0.2, 0.2)

    # Resolve đường dẫn STL
    rp = RosPack()
    pkg_path = rp.get_path("my_table_description")
    mesh_path = os.path.join(pkg_path, "meshes", "table.stl")

    rospy.loginfo(f"Loading mesh from: {mesh_path} with scale {mesh_scale}")
    scene.add_mesh("table", table_pose, mesh_path, mesh_scale)

    # Thiết lập màu nâu gỗ cho object "table"
    color = ObjectColor()
    color.id = "table"
    color.color = ColorRGBA(r=0.8, g=0.6, b=0.4, a=1.0)

    # Tạo PlanningScene diff chứa object_colors
    ps = PlanningScene()
    ps.is_diff = True
    ps.object_colors.append(color)

    # Publish PlanningScene diff lên topic /planning_scene
    pub = rospy.Publisher("planning_scene", PlanningScene, queue_size=1)
    rospy.sleep(1)
    pub.publish(ps)

    rospy.loginfo("Table published and colored in Planning Scene.")
    rospy.spin()

if __name__ == "__main__":
    main()
