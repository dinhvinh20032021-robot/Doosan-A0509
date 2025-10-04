#!/usr/bin/env python3
import sys, os, rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from moveit_msgs.msg import PlanningScene, ObjectColor
from tf.transformations import quaternion_from_euler
from rospkg import RosPack

def set_color(obj_id, color, pub):
    oc = ObjectColor()
    oc.id = obj_id
    oc.color = color
    ps = PlanningScene()
    ps.is_diff = True
    ps.object_colors.append(oc)
    rospy.sleep(0.1)
    pub.publish(ps)

def add_mesh(scene, mesh_name, mesh_path, pose_tuple, rpy=(0,0,0), scale=(1.0,1.0,1.0)):
    p = PoseStamped()
    p.header.frame_id = frame_id
    px, py, pz = pose_tuple
    p.pose.position.x, p.pose.position.y, p.pose.position.z = px, py, pz
    q = quaternion_from_euler(*rpy)
    p.pose.orientation.x, p.pose.orientation.y = q[0], q[1]
    p.pose.orientation.z, p.pose.orientation.w = q[2], q[3]
    scene.add_mesh(mesh_name, p, mesh_path, scale)

def execute_sequence_with_prm(sequence, group):
    # Cấu hình PRM planner
    group.set_planner_id("PRMkConfigDefault")
    group.set_planning_time(30.0)
    group.set_num_planning_attempts(15)
    group.allow_replanning(True)
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)

    for idx, wp in enumerate(sequence):
        rospy.loginfo(f"➡️ Planning to waypoint {idx}")
        group.set_start_state_to_current_state()
        group.set_joint_value_target(wp)

        plan = group.plan()
        plan_msg = plan[0] if isinstance(plan, tuple) else plan

        valid = (plan_msg and hasattr(plan_msg, "joint_trajectory")
                 and len(plan_msg.joint_trajectory.points) > 0)
        if valid:
            rospy.loginfo(f"✅ Executing waypoint {idx}")
            group.execute(plan_msg, wait=True)
        else:
            rospy.logwarn(f"⚠️ Couldn't find a valid PRM plan for waypoint {idx}")

        rospy.sleep(0.5)

if __name__ == "__main__":
    rospy.init_node("move_sequence_prm", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    group = MoveGroupCommander("arm", robot_description="robot_description")
    scene = PlanningSceneInterface()
    pub = rospy.Publisher("planning_scene", PlanningScene, queue_size=1)
    rospy.sleep(1.0)

    frame_id = group.get_planning_frame()
    rp = RosPack()
    pkg = rp.get_path("my_table_description")
    mesh_dir = os.path.join(pkg, "meshes")

    # ================== Thêm vật thể vào scene ==================
    conveyor_fp = os.path.join(mesh_dir, "conveyor.obj")
    add_mesh(scene, "băng_tải", conveyor_fp, (0.0, -0.77, 0.0), scale=(0.004,0.004,0.003))
    set_color("băng_tải", ColorRGBA(1,0,0,1), pub)

    bell_fp = "/home/vinh/Downloads/Bellman_Cart_V1_L1.123cd5dfb12d-39db-4a81-80cf-dff803952d81/18532_Bellman_Cart_V1.obj"
    add_mesh(scene, "xe_khăn_bellman", bell_fp, (0.72, 0.17, 0.0), rpy=(3.14,3.14,1.57), scale=(0.005,0.005,0.004))
    set_color("xe_khăn_bellman", ColorRGBA(1,0.5,0,1), pub)

    service_fp = os.path.join(mesh_dir, "17351_Service_cart_v1.obj")
    add_mesh(scene, "xe_dịch_vụ", service_fp, (-0.77, 0.08, 0.0), rpy=(3.14,-3.14,1.58), scale=(0.005,0.005,0.004))
    set_color("xe_dịch_vụ", ColorRGBA(0,1,1,1), pub)

    ladder_fp = os.path.join(mesh_dir, "18530_Warehouse_Ladder_V1.obj")
    add_mesh(scene, "thang_kho_hàng", ladder_fp, (-1.75, -2.23, 0.0), rpy=(3.14,3.14,1.57), scale=(0.01,0.01,0.008))
    set_color("thang_kho_hàng", ColorRGBA(0.8,0.8,0.8,1), pub)

    ultrasound_fp = os.path.join(mesh_dir, "16978_ultrasound_machine_NEW.obj")
    add_mesh(scene, "máy_siêu_âm", ultrasound_fp, (0.78, -1.58, 0.0), rpy=(0,0,2.54), scale=(0.005,0.005,0.005))
    set_color("máy_siêu_âm", ColorRGBA(1,1,1,1), pub)

    bender_fp = os.path.join(mesh_dir, "16564_Rebar_Bender_Machine_v1.obj")
    add_mesh(scene, "máy_uốn_thép", bender_fp, (-0.40, -1.14, 0.0), rpy=(0,0,0), scale=(0.01,0.01,0.01))
    set_color("máy_uốn_thép", ColorRGBA(0.5,0.5,0.5,1), pub)

    scaffold_fp = os.path.join(mesh_dir, "18529_Scaffolding_Unit_V1.obj")
    add_mesh(scene, "giàn_giáo", scaffold_fp, (-0.44, 0.58, 0.0), rpy=(3.14,-3.14,-3.14), scale=(0.004,0.004,0.004))
    set_color("giàn_giáo", ColorRGBA(0.8,0.8,0.8,1), pub)

    oscilloscope_fp = os.path.join(mesh_dir, "16959_oscilloscope_NEW.obj")
    add_mesh(scene, "máy_đo_oscilloscope", oscilloscope_fp, (0.80,0.18,0.11), rpy=(3.14,3.14,-1.52), scale=(0.01,0.01,0.01))
    set_color("máy_đo_oscilloscope", ColorRGBA(0,1,0,1), pub)

    hemostat_fp = os.path.join(mesh_dir, "17225_Hemostat_V1.obj")
    add_mesh(scene, "kẹp_huyết", hemostat_fp, (-0.24, -0.47, 0.41), rpy=(0,0,0), scale=(0.01,0.01,0.01))
    set_color("kẹp_huyết", ColorRGBA(0,1,0,1), pub)

    nail_fp = os.path.join(mesh_dir, "20241_Nail_(fastener)_V1.obj")
    add_mesh(scene, "đinh", nail_fp, (-0.55, 0.08, 0.45), rpy=(0,0,0), scale=(0.01,0.01,0.01))
    set_color("đinh", ColorRGBA(0,1,0,1), pub)

    nailgun_fp = os.path.join(mesh_dir, "17731_Nail_Gun_v1.obj")
    add_mesh(scene, "súng_bắn_đinh", nailgun_fp, (0.34, -0.54, 0.40), rpy=(1.49,0.05,3.12), scale=(0.005,0.005,0.005))
    set_color("súng_bắn_đinh", ColorRGBA(0,1,0,1), pub)

    rospy.loginfo("✅ Tất cả vật thể đã được thêm vào MoveIt scene")

    # ================== Sequence waypoint ==================
    sequence = [
        [-1.454, 0.266, 1.233, -0.039, 1.567, 0.010],   # goal1
        [-3.331, 0.438, 0.931, -3.137, -1.778, -2.004],  # goal2
        [-5.153, 0.437, 0.938, -3.137, -1.772, -2.044], # goal3
        [-2.945, -0.769, -1.082, 3.094, 1.222, 0.177], # goal4
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]                  # home
    ]

    # ================== Thực thi sequence dùng PRM ==================
    execute_sequence_with_prm(sequence, group)

    rospy.loginfo("🎉 Sequence với PRM đã hoàn tất")
    moveit_commander.roscpp_shutdown()
"""
BẢNG CHÚ THÍCH VẬT THỂ:

conveyor            → băng_tải
bellman_cart         → xe_khăn_bellman
service_cart         → xe_dịch_vụ
warehouse_ladder     → thang_kho_hàng
ultrasound_machine   → máy_siêu_âm
rebar_bender         → máy_uốn_thép
scaffolding_unit     → giàn_giáo
oscilloscope         → máy_đo_oscilloscope
hemostat             → kẹp_huyết
nail_fastener        → đinh
nail_gun             → súng_bắn_đinh
"""
