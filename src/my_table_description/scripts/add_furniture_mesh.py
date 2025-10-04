#!/usr/bin/env python3
import sys, os, rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface, RobotCommander, MoveGroupCommander
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

def execute_joint_sequence(joint_seq, group):
    # === Sử dụng planner RRTConnect ===
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_planning_time(30.0)
    group.set_num_planning_attempts(10)
    group.allow_replanning(True)
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)

    for idx, joints in enumerate(joint_seq):
        rospy.loginfo(f"➡️ Moving to waypoint {idx}: {joints}")
        # luôn set start state hiện tại
        group.set_start_state_to_current_state()
        group.set_joint_value_target(joints)
        plan = group.plan()

        plan_msg = plan[0] if isinstance(plan, tuple) else plan
        valid = (plan_msg and hasattr(plan_msg, "joint_trajectory")
                 and len(plan_msg.joint_trajectory.points) > 0)

        if not valid:
            rospy.logwarn("⚠️ Plan empty — retrying with more time")
            group.set_planning_time(60.0)  # tăng thêm thời gian
            plan = group.plan()
            plan_msg = plan[0] if isinstance(plan, tuple) else plan
            valid = (plan_msg and hasattr(plan_msg, "joint_trajectory")
                     and len(plan_msg.joint_trajectory.points) > 0)

        if valid:
            ok = group.execute(plan_msg, wait=True)
            if ok:
                rospy.loginfo(f"✅ Reached waypoint {idx}")
            else:
                rospy.logerr(f"❌ Execution failed at waypoint {idx}")
                return False
        else:
            rospy.logerr(f"❌ Couldn't find a valid plan for waypoint {idx}")
            return False

        rospy.sleep(0.5)
    return True

if __name__ == "__main__":
    rospy.init_node("env_rrtconnect_sequence", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    robot = RobotCommander("robot_description")
    group = MoveGroupCommander("arm", robot_description="robot_description")
    scene = PlanningSceneInterface()
    pub = rospy.Publisher("planning_scene", PlanningScene, queue_size=1)
    rospy.sleep(1.0)

    frame_id = group.get_planning_frame()
    rp = RosPack()
    pkg = rp.get_path("my_table_description")
    mesh_dir = os.path.join(pkg, "meshes")

    # ================== Thêm môi trường ==================
    conveyor_fp = os.path.join(mesh_dir, "conveyor.obj")
    add_mesh(scene, "conveyor", conveyor_fp, (0.0, -0.77, 0.0), scale=(0.004,0.004,0.003))
    set_color("conveyor", ColorRGBA(1,0,0,1), pub)

    cart_fp = os.path.join(mesh_dir, "18533_Utility_Cart_V1.obj")
    add_mesh(scene, "utility_cart", cart_fp, (0.25, 0.75, 0.01), rpy=(0.00,0.00,1.58), scale=(0.0667,0.0667,0.05))
    set_color("utility_cart", ColorRGBA(0,0.3,0.8,1), pub)

    bell_fp = "/home/vinh/Downloads/Bellman_Cart_V1_L1.123cd5dfb12d-39db-4a81-80cf-dff803952d81/18532_Bellman_Cart_V1.obj"
    add_mesh(scene, "bellman_cart", bell_fp, (0.72, 0.17, 0.0), rpy=(3.14,3.14,1.57), scale=(0.005,0.005,0.004))
    set_color("bellman_cart", ColorRGBA(1,0.5,0,1), pub)

    service_fp = os.path.join(mesh_dir, "17351_Service_cart_v1.obj")
    add_mesh(scene, "service_cart", service_fp, (-0.77, 0.08, 0.0), rpy=(3.14,-3.14,1.58), scale=(0.005,0.005,0.004))
    set_color("service_cart", ColorRGBA(0,1,1,1), pub)

    ladder_fp = os.path.join(mesh_dir, "18530_Warehouse_Ladder_V1.obj")
    add_mesh(scene, "warehouse_ladder", ladder_fp, (-1.75, -2.23, 0.0), rpy=(3.14,3.14,1.57), scale=(0.01,0.01,0.008))
    set_color("warehouse_ladder", ColorRGBA(0.8,0.8,0.8,1), pub)

    ultrasound_fp = os.path.join(mesh_dir, "16978_ultrasound_machine_NEW.obj")
    add_mesh(scene, "ultrasound_machine", ultrasound_fp, (0.78, -1.58, 0.0), rpy=(0,0,2.54), scale=(0.005,0.005,0.005))
    set_color("ultrasound_machine", ColorRGBA(1,1,1,1), pub)

    bender_fp = os.path.join(mesh_dir, "16564_Rebar_Bender_Machine_v1.obj")
    add_mesh(scene, "rebar_bender", bender_fp, (-0.40, -1.14, 0.0), rpy=(0,0,0), scale=(0.01,0.01,0.01))
    set_color("rebar_bender", ColorRGBA(0.5,0.5,0.5,1), pub)

    scaffold_fp = os.path.join(mesh_dir, "18529_Scaffolding_Unit_V1.obj")
    add_mesh(scene, "scaffolding_unit", scaffold_fp, (-0.44, 0.58, 0.0), rpy=(3.14,-3.14,-3.14), scale=(0.004,0.004,0.004))
    set_color("scaffolding_unit", ColorRGBA(0.8,0.8,0.8,1), pub)

    oscilloscope_fp = os.path.join(mesh_dir, "16959_oscilloscope_NEW.obj")
    add_mesh(scene, "oscilloscope", oscilloscope_fp, (0.80, 0.18, 0.11), rpy=(3.14,3.14,-1.52), scale=(0.01,0.01,0.01))
    set_color("oscilloscope", ColorRGBA(0,1,0,1), pub)

    hemostat_fp = os.path.join(mesh_dir, "17225_Hemostat_V1.obj")
    add_mesh(scene, "hemostat", hemostat_fp, (-0.24, -0.47, 0.41), rpy=(0,0,0), scale=(0.01,0.01,0.01))
    set_color("hemostat", ColorRGBA(0,1,0,1), pub)

    nail_fp = os.path.join(mesh_dir, "20241_Nail_(fastener)_V1.obj")
    add_mesh(scene, "nail_fastener", nail_fp, (-0.55, 0.08, 0.45), rpy=(0,0,0), scale=(0.01,0.01,0.01))
    set_color("nail_fastener", ColorRGBA(0,1,0,1), pub)

    catpaw_fp = os.path.join(mesh_dir, "18787_Cat's_paw_(nail_puller)_V1.obj")
    add_mesh(scene, "catpaw", catpaw_fp, (0.23, 0.55, 0.44), rpy=(0.06,-0.07,1.62), scale=(0.002,0.002,0.002))
    set_color("catpaw", ColorRGBA(0,1,0,1), pub)

    nailgun_fp = os.path.join(mesh_dir, "17731_Nail_Gun_v1.obj")
    add_mesh(scene, "nail_gun", nailgun_fp, (0.34, -0.54, 0.40), rpy=(1.49,0.05,3.12), scale=(0.005,0.005,0.005))
    set_color("nail_gun", ColorRGBA(0,1,0,1), pub)

    rospy.loginfo("✅ All objects added to MoveIt scene")

    # ================== Sequence các điểm ==================
    seq = [
        [-1.454, 0.266, 1.233, -0.039, 1.567, 0.010],   # goal1
        [-2.945, -0.769, -1.082, 3.094, 1.222, 0.177],   # goal2
        [-5.153, 0.437, 0.938, -3.137, -1.772, -2.044],   # goal3
        [-3.331, 0.438, 0.931, -3.137, -1.778, -2.004],     # goal4
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]                   # về home cuối
    ]
    ok = execute_joint_sequence(seq, group)
    if ok:
        rospy.loginfo("🎉 Sequence completed successfully with RRTConnect")
    else:
        rospy.logerr("⚠️ Sequence failed")

    moveit_commander.roscpp_shutdown()
