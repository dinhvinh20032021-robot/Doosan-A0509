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

def add_mesh(scene, name, filepath, position, rpy=(0,0,0), scale=(1,1,1)):
    try:
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        quat = tf_trans.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
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

def main():
    rospy.init_node('rrt_start', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    group_name = "arm"
    group = MoveGroupCommander(group_name)
    group.set_planner_id("RRTStart")
    group.set_max_velocity_scaling_factor(0.5)
    group.set_max_acceleration_scaling_factor(0.5)

    scene = PlanningSceneInterface()
    rospy.sleep(1)
    pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
    rospy.sleep(1)

    # Thêm mesh (chia rõ nhiều dòng)
    mesh_dir = "/home/vinh/catkin_ws/src/my_table_description/meshes"

    meshes = [
        (
            "băng_tải",
            os.path.join(mesh_dir, "conveyor.obj"),
            (0.0, -0.77, 0.0),
            (0, 0, 0),
            (0.004, 0.004, 0.003),
            ColorRGBA(1, 0, 0, 1)
        ),
        (
            "xe_khăn_bellman",
            "/home/vinh/Downloads/Bellman_Cart_V1_L1.123cd5dfb12d-39db-4a81-80cf-dff803952d81/18532_Bellman_Cart_V1.obj",
            (0.72, 0.17, 0.0),
            (3.14, 3.14, 1.57),
            (0.005, 0.005, 0.004),
            ColorRGBA(1, 0.5, 0, 1)
        ),
        (
            "utility_cart",
            os.path.join(mesh_dir, "18533_Utility_Cart_V1.obj"),
            (0.25, 0.75, 0.01),
            (0, 0, 1.58),
            (0.0667, 0.0667, 0.05),
            ColorRGBA(0, 0.3, 0.8, 1)
        ),
        (
            "service_cart",
            os.path.join(mesh_dir, "17351_Service_cart_v1.obj"),
            (-0.77, 0.08, 0.0),
            (3.14, -3.14, 1.58),
            (0.005, 0.005, 0.004),
            ColorRGBA(0, 1, 1, 1)
        ),
        (
            "scaffolding_unit",
            os.path.join(mesh_dir, "18529_Scaffolding_Unit_V1.obj"),
            (-0.44, 0.58, 0.0),
            (3.14, -3.14, -3.14),
            (0.004, 0.004, 0.004),
            ColorRGBA(0.8, 0.8, 0.8, 1)
        ),
        (
            "cordless_drill",
            os.path.join(mesh_dir, "10108_Cordless Drill_v2_Iteration2.obj"),
            (-0.69, 0.06, 0.45),
            (0.04, 0.03, 1.67),
            (0.01, 0.01, 0.01),
            ColorRGBA(0.3, 1.0, 0.3, 1)
        ),
        (
            "oscilloscope",
            os.path.join(mesh_dir, "16959_oscilloscope_NEW.obj"),
            (0.80, 0.18, 0.11),
            (3.14, 3.14, -1.52),
            (0.01, 0.01, 0.01),
            ColorRGBA(0, 1, 0, 1)
        ),
        (
            "catpaw",
            os.path.join(mesh_dir, "18787_Cat's_paw_(nail_puller)_V1.obj"),
            (0.23, 0.55, 0.44),
            (0.06, -0.07, 1.62),
            (0.002, 0.002, 0.002),
            ColorRGBA(0, 1, 0, 1)
        ),
        (
            "nail_gun",
            os.path.join(mesh_dir, "17731_Nail_Gun_v1.obj"),
            (-0.08, -0.74, 0.42),
            (1.65, -3.09, 0.1),
            (0.005, 0.005, 0.005),
            ColorRGBA(0, 1, 0, 1)
        ),
        (
            "finger_nail_bottle",
            os.path.join(mesh_dir, "16849_finger_nail_paint_bottle_v1_NEW.obj"),
            (0.28, 0.60, 0.46),
            (0, 0, 0),
            (0.03, 0.03, 0.03),
            ColorRGBA(1, 0.5, 0, 1)
        ),
    ]

    for name, fp, pos, rpy, scale, color in meshes:
        if add_mesh(scene, name, fp, pos, rpy, scale):
            set_color(name, color, pub)

    # Chuỗi joint goals
    seq = [
        [-1.454, 0.266, 1.233, -0.039, 1.567, 0.010],
        [-2.945, -0.769, -1.082, 3.094, 1.222, 0.177],
        [-5.153, 0.437, 0.938, -3.137, -1.772, -2.044],
        [-3.331, 0.438, 0.931, -3.137, -1.778, -2.004],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ]

    # Thực thi lần lượt các goal
    for i, joint_goal in enumerate(seq):
        try:
            rospy.loginfo(f"[{i+1}/{len(seq)}] Planning goal {i+1}")
            plan_result = group.plan(joint_goal)

            plan = None
            if hasattr(plan_result, 'joint_trajectory'):
                plan = plan_result
            elif isinstance(plan_result, tuple):
                for item in plan_result:
                    if hasattr(item, 'joint_trajectory'):
                        plan = item
                        break

            if plan and hasattr(plan, 'joint_trajectory') and len(plan.joint_trajectory.points) > 0:
                rospy.loginfo(f"Goal {i+1} plan thành công, points={len(plan.joint_trajectory.points)}")
                success = group.execute(plan, wait=True)
                group.stop()
                rospy.loginfo(f"Goal {i+1} execute {'thành công' if success else 'thất bại'}")
            else:
                rospy.logwarn(f"Goal {i+1} không thể plan được hoặc trajectory rỗng")
        except Exception as e:
            rospy.logerr(f"Goal {i+1} lỗi: {e}")

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
