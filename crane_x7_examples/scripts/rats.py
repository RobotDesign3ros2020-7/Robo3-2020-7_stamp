#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("pose_groupstate_example")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.75)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # SRDFに定義されている"home"の姿勢にする
    def home_pos():
        print("home")
        arm.set_named_target("home")
        arm.go()

    # SRDFに定義されている"vertical"の姿勢にする
    def vertical_pos():
        print("vertical")
        arm.set_named_target("vertical")
        arm.go()

    # ハンドを少し閉じる
    def open_close(per):
        gripper.set_joint_value_target([per,per])
        gripper.go()

    # 手動で姿勢を指定するには以下のように指定
    def set_pos(set_x,set_y,set_z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = set_x
        target_pose.position.y = set_y
        target_pose.position.z = set_z
        q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target( target_pose )	# 目標ポーズ設定
        arm.go()				# 実行

    def set_pos_1(set_x,set_y,set_z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = set_x
        target_pose.position.y = set_y
        target_pose.position.z = set_z
        q = quaternion_from_euler( -3.14/2.0, 0.0, -3.14/2.0 )
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target( target_pose )	# 目標ポーズ設定
        arm.go()				# 実行


    home_pos()

    set_pos(0.33,0.,0.23)
    set_pos(0.33,0.15,0.23)
    set_pos(0.33,0.15,0.13)

    open_close(0.23)

    set_pos(0.33,0.15,0.23)
    set_pos(0.33,0.,0.23)
    set_pos(0.33,-0.15,0.23)
    set_pos(0.33,-0.15,0.13)
    set_pos(0.33,-0.15,0.18)
    set_pos(0.33,-0.15,0.13)
    set_pos(0.33,-0.15,0.18)

    set_pos(0.22,0.,0.18)
    set_pos(0.22,0.,0.13)
    set_pos(0.20,0.,0.13)
    set_pos(0.20,0.,0.11)
    set_pos(0.20,0.01,0.11)
    set_pos(0.22,0.,0.18)

    set_pos(0.33,0.15,0.18)
    set_pos(0.33,0.15,0.13)


    open_close(0.7)

    home_pos()
    vertical_pos()

    # 移動後の手先ポーズを表示
    arm_goal_pose = arm.get_current_pose().pose
    print("Arm goal pose:")
    print(arm_goal_pose)
    print("done")


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
