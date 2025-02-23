#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg

class PathFollower:
    def __init__(self):
        # Initialize MoveIt!
        moveit_commander.roscpp_initialize([])
        rospy.init_node('path_follower', anonymous=True)

        # MoveIt! interfaces
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("left_arm")  # Change "arm" to your move group

        # Set MoveIt! parameters
        self.group.set_planning_time(5)
        self.group.set_max_velocity_scaling_factor(0.5)
        self.group.set_max_acceleration_scaling_factor(0.5)

    def follow_path(self, waypoints):
        """Plans and executes a path using Cartesian interpolation."""
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,   # list of waypoints
            0.01,        # eef_step: resolution of the path
            0.0          # jump_threshold: set 0 to disable
        )

        if fraction < 1.0:
            rospy.logwarn("Only {:.2f} of the path was planned.".format(fraction * 100))

        self.group.execute(plan, wait=True)

    def run(self):
        """Define and follow waypoints"""
        waypoints = []

        # Define waypoints (adjust according to your robot)
        pose1 = self.group.get_current_pose().pose
        pose1.position.x += 0.1
        waypoints.append(pose1)

        pose2 = geometry_msgs.msg.Pose()
        pose2.position.x = pose1.position.x
        pose2.position.y = pose1.position.y + 0.1
        pose2.position.z = pose1.position.z
        waypoints.append(pose2)

        pose3 = geometry_msgs.msg.Pose()
        pose3.position.x = pose2.position.x - 0.1
        pose3.position.y = pose2.position.y
        pose3.position.z = pose2.position.z
        waypoints.append(pose3)

        rospy.loginfo("Executing path...")
        self.follow_path(waypoints)

if __name__ == "__main__":
    follower = PathFollower()
    follower.run()
