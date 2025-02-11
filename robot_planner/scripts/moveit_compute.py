#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

# Initialize MoveIt!
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_robot_control', anonymous=True)

# Create MoveGroupCommander for the robot arm
group_name = "manipulator"  # Replace with your actual planning group name
move_group = moveit_commander.MoveGroupCommander(group_name)

# Move to a predefined pose
move_group.set_named_target("home")  # Replace "home" with your actual pose
plan = move_group.go(wait=True)
move_group.stop()

# Shutdown MoveIt!
moveit_commander.roscpp_shutdown()
