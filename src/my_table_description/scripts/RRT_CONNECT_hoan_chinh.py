#!/usr/bin/env python3
import sys
import os
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import PlanningScene, ObjectColor
import tf.transformations as tf_trans
import time

# ============================================================
# HÀM HỖ TRỢ
# ============================================================
def add_mesh(scene, name, filepath, position, rpy=(0,0,0), scale=(1,1,1)):
    try:
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        quat = tf_trans.quaternion_from_euler(*rpy)
        pose.pose.orientation = Quaternion(*quat)

        scene.add_mesh(name, pose, filepath, size=scale)
        rospy.loginfo(f"Mesh '{name}' đã được thêm vào scene thành công")
        return True
    except Exception as e:
        rospy.logerr(f"Lỗi khi thêm mesh '{name}': {e}")
        return False

def set_color(name, color, pub):
    try:
        oc = ObjectColor()
        oc.id = name
        oc.color = color
        ps = PlanningScene()
        ps.is_diff = True
        ps.object_colors.append(oc)
        pub.publish(ps)
        rospy.loginfo(f"Màu sắc cho '{name}' đã được thiết lập")
        return True
    except Exception as e:
        rospy.logerr(f"Lỗi khi thiết lập màu cho '{name}': {e}")
        return False

def wait_for_robot_description(ns, timeout=10.0):
    param_name = f"{ns}/robot_description"
    t0 = time.time()
    rospy.loginfo(f"Chờ param '{param_name}' trên parameter server...")
    while time.time() - t0 < timeout and not rospy.is_shutdown():
        if rospy.has_param(param_name):
            rospy.loginfo(f"Found param '{param_name}'")
            return True
        rospy.sleep(0.1)
    rospy.logwarn(f"Không tìm thấy param '{param_name}' sau {timeout}s")
    return False

# ============================================================
# MAIN
# ============================================================
def main():
    rospy.init_node('rrt_start', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    # ------ CONFIG ------
    ns = rospy.get_param('~ns', '/dsr01a0509')
    robot_desc_param = f"{ns}/robot_description"
    wait_for_robot_description(ns, timeout=10.0)

    group_name = "arm"
    group = MoveGroupCommander(group_name, robot_description=robot_desc_param, ns=ns)
    scene = PlanningSceneInterface(ns=ns, synchronous=True)

    planning_scene_topic = f"{ns}/planning_scene"
    pub = rospy.Publisher(planning_scene_topic, PlanningScene, queue_size=10)
    rospy.sleep(1)
    # --------------------

    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)

    # ========================================================
    # DANH SÁCH MESH - dễ đọc
    # ========================================================
    mesh_dir = "/home/vinh/catkin_ws/src/my_table_description/meshes"

    mesh_config = [
        {
            "name": "băng_tải",
            "file": os.path.join(mesh_dir, "conveyor.obj"),
            "pos": (0.0, -0.77, 0.0),
            "rpy": (0,0,0),
            "scale": (0.004,0.004,0.003),
            "color": ColorRGBA(1,0,0,1)
        },
        {
            "name": "xe_khăn_bellman",
            "file": "/home/vinh/Downloads/Bellman_Cart_V1_L1.123cd5dfb12d-39db-4a81-80cf-dff803952d81/18532_Bellman_Cart_V1.obj",
            "pos": (0.72, 0.17, 0.0),
            "rpy": (3.14,3.14,1.57),
            "scale": (0.005,0.005,0.004),
            "color": ColorRGBA(1,0.5,0,1)
        },
        {
            "name": "utility_cart",
            "file": os.path.join(mesh_dir, "18533_Utility_Cart_V1.obj"),
            "pos": (0.25, 0.75, 0.01),
            "rpy": (0,0,1.58),
            "scale": (0.0667,0.0667,0.05),
            "color": ColorRGBA(0,0.3,0.8,1)
        },
        {
            "name": "service_cart",
            "file": os.path.join(mesh_dir, "17351_Service_cart_v1.obj"),
            "pos": (-0.77, 0.08, 0.0),
            "rpy": (3.14,-3.14,1.58),
            "scale": (0.005,0.005,0.004),
            "color": ColorRGBA(0,1,1,1)
        },
        {
            "name": "scaffolding_unit",
            "file": os.path.join(mesh_dir, "18529_Scaffolding_Unit_V1.obj"),
            "pos": (-0.44, 0.58, 0.0),
            "rpy": (3.14,-3.14,-3.14),
            "scale": (0.004,0.004,0.004),
            "color": ColorRGBA(0.8,0.8,0.8,1)
        },
        {
            "name": "cordless_drill",
            "file": os.path.join(mesh_dir, "10108_Cordless Drill_v2_Iteration2.obj"),
            "pos": (-0.70, 0.07, 0.45),
            "rpy": (3.11,3.10,-1.47),
            "scale": (0.01,0.01,0.01),
            "color": ColorRGBA(0.3,1.0,0.3,1)
        },
        {
            "name": "oscilloscope",
            "file": os.path.join(mesh_dir, "16959_oscilloscope_NEW.obj"),
            "pos": (0.80, 0.18, 0.11),
            "rpy": (3.14,3.14,-1.52),
            "scale": (0.01,0.01,0.01),
            "color": ColorRGBA(0,1,0,1)
        },
        {
            "name": "catpaw",
            "file": os.path.join(mesh_dir, "18787_Cat's_paw_(nail_puller)_V1.obj"),
            "pos": (0.23, 0.55, 0.44),
            "rpy": (0.06,-0.07,1.62),
            "scale": (0.002,0.002,0.002),
            "color": ColorRGBA(0,1,0,1)
        },
        {
            "name": "nail_gun",
            "file": os.path.join(mesh_dir, "17731_Nail_Gun_v1.obj"),
            "pos": (-0.08, -0.74, 0.42),
            "rpy": (1.65,-3.09,0.1),
            "scale": (0.005,0.005,0.005),
            "color": ColorRGBA(0,1,0,1)
        },
        {
            "name": "finger_nail_bottle",
            "file": os.path.join(mesh_dir, "16849_finger_nail_paint_bottle_v1_NEW.obj"),
            "pos": (0.28, 0.60, 0.46),
            "rpy": (0,0,0),
            "scale": (0.03,0.03,0.03),
            "color": ColorRGBA(1,0.5,0,1)
        }
    ]

    # Thêm mesh vào scene
    for cfg in mesh_config:
        if add_mesh(scene, cfg["name"], cfg["file"], cfg["pos"], cfg["rpy"], cfg["scale"]):
            set_color(cfg["name"], cfg["color"], pub)

    # ========================================================
    # JOINT GOALS
    # ========================================================
    seq = [
        [-1.454, 0.266, 1.233, -0.039, 1.567, 0.010],
        [-2.945, -0.769, -1.082, 3.094, 1.222, 0.177],
        [-5.153, 0.437, 0.938, -3.137, -1.772, -2.044],
        [-3.331, 0.438, 0.931, -3.137, -1.778, -2.004],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ]

    for i, joint_goal in enumerate(seq):
        try:
            rospy.loginfo(f"[{i+1}/{len(seq)}] Planning goal {i+1}")
            plan = group.plan(joint_goal)

            valid_plan = None
            if hasattr(plan, 'joint_trajectory'):
                valid_plan = plan
            elif isinstance(plan, tuple):
                for item in plan:
                    if hasattr(item, 'joint_trajectory'):
                        valid_plan = item
                        break

            if valid_plan and len(valid_plan.joint_trajectory.points) > 0:
                rospy.loginfo(f"Goal {i+1} plan thành công, points={len(valid_plan.joint_trajectory.points)}")
                success = group.execute(valid_plan, wait=True)
                group.stop()
                rospy.loginfo(f"Goal {i+1} execute {'OK' if success else 'FAIL'}")
            else:
                rospy.logwarn(f"Goal {i+1} không thể plan được hoặc trajectory rỗng")

        except Exception as e:
            rospy.logerr(f"Goal {i+1} lỗi: {e}")

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
