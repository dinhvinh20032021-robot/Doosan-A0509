#!/usr/bin/env python3

import sys
import os
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from moveit_msgs.msg import PlanningScene, ObjectColor
from tf.transformations import quaternion_from_euler
from rospkg import RosPack

def add_box(scene, name, size, position, rpy, frame_id):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
    q = quaternion_from_euler(*rpy)
    pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
    pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
    scene.add_box(name, pose, size=size)
    rospy.loginfo(f"✅ Added obstacle '{name}' at {position}")

def add_table(scene, frame_id):
    # Load mesh
    rp       = RosPack()
    pkg_path = rp.get_path("my_table_description")
    mesh_fp  = os.path.join(pkg_path, "meshes", "table.stl")

    # Pose cho table
    table_pose = PoseStamped()
    table_pose.header.frame_id = frame_id
    table_pose.pose.position.x = -0.4
    table_pose.pose.position.y =  2.20
    table_pose.pose.position.z =  0.04
    table_pose.pose.orientation.w = 1.0

    # Scale table
    mesh_scale = (0.2, 0.2, 0.2)
    scene.add_mesh("table", table_pose, mesh_fp, mesh_scale)
    rospy.loginfo("✅ Added table mesh")

    # --- Publish PlanningScene diff để set màu grey cho table ---
    color = ObjectColor()
    color.id = "table"
    color.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)

    ps = PlanningScene()
    ps.is_diff = True
    ps.object_colors.append(color)

    pub = rospy.Publisher("planning_scene", PlanningScene, queue_size=1)
    rospy.sleep(0.5)
    pub.publish(ps)
    rospy.loginfo("🎨 Table colored grey")

def main():
    rospy.init_node("move_sequence_rrt", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander(
        robot_description="/dsr01m1013/robot_description", ns="/dsr01m1013"
    )
    group = moveit_commander.MoveGroupCommander(
        "arm", robot_description="/dsr01m1013/robot_description", ns="/dsr01m1013"
    )
    scene = PlanningSceneInterface(ns="/dsr01m1013")

    rospy.sleep(1.0)
    frame_id = group.get_planning_frame()
    rospy.loginfo(f"Planning frame: {frame_id}")

    # 1) Add table + color
    add_table(scene, frame_id)

    # 2) Add 19 hộp obstacles
    boxes = [
      ("box1",(0.2,0.2,0.2),( 0.11,-0.78,0.13)),
      ("box2",(0.2,0.2,0.2),( 0.60,-0.34,0.73)),
      ("box3",(0.2,0.2,0.2),(-0.86,-0.89,0.10)),
      ("box4",(0.2,0.2,0.2),( 0.66, 0.31,1.08)),
      ("box5",(0.2,0.2,0.2),( 0.12,-0.79,0.45)),
      ("box6",(0.2,0.2,0.2),(-0.84, 0.00,0.11)),
      ("box7",(0.2,0.2,0.2),(-0.84, 0.00,0.41)),
      ("box8",(0.2,0.2,0.2),(-0.68, 0.00,1.20)),
      ("box9",(0.2,0.2,0.2),( 0.00, 0.94,0.12)),
      ("box10",(0.2,0.2,0.2),(-0.92, 0.76,0.11)),
      ("box11",(0.2,0.2,0.2),(-0.96, 0.79,0.43)),
      ("box12",(0.2,0.2,0.2),(-0.70,-0.76,0.41)),
      ("box13",(0.2,0.2,0.2),(-0.27, 0.72,0.54)),
      ("box14",(0.2,0.2,0.2),( 0.00,-0.52,1.02)),
      ("box15",(0.2,0.2,0.2),(-0.11, 0.76,1.09)),
      ("box16",(0.2,0.2,0.2),( 0.84, 0.51,0.12)),
      ("box17",(0.2,0.2,0.2),( 0.72,-0.27,0.09)),
      ("box18",(0.2,0.2,0.2),( 0.61, 0.10,0.52)),
      ("box19",(0.2,0.2,0.2),( 0.00, 0.09,1.67)),
    ]
    for name, size, pos in boxes:
        add_box(scene, name, size, pos, (0,0,0), frame_id)

    # Đợi tất cả object vào scene
    expected = {n for n, *_ in boxes} | {"table"}
    t0 = rospy.get_time()
    while not rospy.is_shutdown() and rospy.get_time() - t0 < 5.0:
        if expected.issubset(set(scene.get_known_object_names())):
            rospy.loginfo("✅ All objects are now in the scene")
            break
        rospy.sleep(0.1)

    # 3) Cấu hình RRT & chạy motion
    group.set_goal_joint_tolerance(0.01)
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)
    group.set_planning_time(10.0)
    group.set_planner_id("RRTkConfigDefault")

    home  = [0,0,0,0,0,0]
    goal1 = [-2.805,0.619,1.374,-0.004,1.191,-3.252]
    goal2 = [-3.850,0.619,1.374,-0.004,1.191,-3.252]
    seq   = [home,goal1,goal2,home]

    rospy.loginfo("🚀 Executing motion using RRT")
    for idx, tgt in enumerate(seq):
        group.set_joint_value_target(tgt)
        plan = group.plan()
        if plan and plan[0]:
            rospy.loginfo(f"➡️ Moving to point {idx+1}")
            group.execute(plan[1], wait=True)
            group.stop()
            group.clear_pose_targets()
            rospy.sleep(1.0)
        else:
            rospy.logwarn(f"⚠️ Planning failed at point {idx+1}")
            break

    rospy.loginfo("✅ Motion complete.")
    moveit_commander.roscpp_shutdown()

if __name__=="__main__":
    main()
