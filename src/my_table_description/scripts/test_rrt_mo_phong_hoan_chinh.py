#!/usr/bin/env python3
import sys, rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from tf.transformations import quaternion_from_euler

def execute_sequence_with_rrt(sequence, group):
    # Cấu hình RRT planner
    group.set_planner_id("RRTConnectkConfigDefault")  # hoặc "RRTstarkConfigDefault" nếu muốn
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
            rospy.logwarn(f"⚠️ Couldn't find a valid RRT plan for waypoint {idx}")

        rospy.sleep(0.5)

if __name__ == "__main__":
    rospy.init_node("move_sequence_rrt", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    group = MoveGroupCommander("arm", robot_description="robot_description")
    rospy.sleep(1.0)

    # ================== Sequence waypoint ==================
    sequence = [
           # goal1
         # goal2
         # goal3
        [-3.331, 0.438, 0.931, -3.137, -1.778, -2.004], # goal4
                         # home
    ]

    # ================== Thực thi sequence dùng RRT ==================
    execute_sequence_with_rrt(sequence, group)

    rospy.loginfo("🎉 Sequence with RRT completed")
    moveit_commander.roscpp_shutdown()
