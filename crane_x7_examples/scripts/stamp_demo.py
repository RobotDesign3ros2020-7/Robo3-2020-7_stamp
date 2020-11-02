#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler

def target_pose(x, y, z):
    arm = moveit_commander.MoveGroupCommander("arm")
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

def target_joint_values(a, b):
    arm = moveit_commander.MoveGroupCommander("arm")
    target_joint_values = arm.get_current_joint_values()
    target_joint_values[a] = math.radians(b)
    arm.set_joint_value_target(target_joint_values)    # 目標ポーズ設定
    arm.go()    # 実行

def main():	
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.4)
    PICK_Z = 0.12                   # 掴む時のハンドの高さ
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
    arm.set_named_target("home")
    arm.go()
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()

    # 掴む準備をする
    target_pose(0.2, 0.0, 0.3)

    # ハンドを開く
    gripper.set_joint_value_target([0.8, 0.8])
    gripper.go()

    # 掴みに行く
    target_pose(0.2, 0.0, PICK_Z)

    # ハンドを閉じる
    gripper.set_joint_value_target([0.4, 0.4])
    gripper.go()

    # 持ち上げる
    target_pose(0.2, 0.0, 0.3)

    # 移動する 朱肉まで
#    target_pose(0.2, 0.2, 0.3)

    # 下ろす 朱肉をつける
#    target_pose(0.2, 0.2, PICK_Z-0.02)

    # 少しだけハンドを持ち上げる
#    target_pose(0.2, 0.2, 0.15)
    
    # 下ろす 朱肉をつける2回目
#    target_pose(0.2, 0.2, PICK_Z-0.02)
    
    # ハンドを持ち上げる
#    target_pose(0.2, 0.2, 0.3)
    
    # 移動する 捺印場所
    target_pose(0.3, 0.1, 0.3)
    rospy.sleep(1.0)
    
    # 下ろす 捺印
    target_pose(0.3, 0.1, PICK_Z-0.02)
    
    # 角度変更
    target_joint_values(6, -0.785398)
    rospy.sleep(1.0)
#    target_joint_values(6, -10)
    rospy.sleep(1.0)
    
    # ハンドを持ち上げる
    target_pose(0.3, 0.1, 0.3)
    rospy.sleep(1.0)
    
    # 移動する ホームポジション
    target_pose(0.2, 0.0, 0.3)
    
    # 下ろす ホームポジション
    target_pose(0.2, 0.0, PICK_Z)
    
    # ハンドを開く
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()
    
    # 少しだけハンドを持ち上げる
    target_pose(0.2, 0.0, 0.2)

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()

    print("done")


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
