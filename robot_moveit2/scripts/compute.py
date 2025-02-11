#!/usr/bin/env python3

import rospy
import moveit_commander
import numpy as np
import csv
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose

# Initialize MoveIt! and ROS
rospy.init_node('reachable_workspace_computation', anonymous=True)
moveit_commander.roscpp_initialize([])

# Define MoveGroupCommander for your robot arm
group_name = "all_joints"  # Change this to your MoveIt! group name
move_group = moveit_commander.MoveGroupCommander(group_name)

# Get the end effector link name
end_effector_link = move_group.get_end_effector_link()
if not end_effector_link:
    rospy.logerr("No end effector link found!")
    exit()

rospy.loginfo(f"Using End Effector Link: {end_effector_link}")

# Retrieve joint limits
joint_limits = move_group.get_joints()
joint_limits = [move_group.get_joint_limits(joint) for joint in joint_limits if move_group.has_joint(joint)]

# Set number of samples
num_samples = 10000  # Increase for better resolution
reachable_points = []

# Randomly sample joint configurations and compute end-effector positions
rospy.loginfo("Sampling joint space and computing reachable workspace...")
for _ in range(num_samples):
    random_joint_values = [np.random.uniform(joint_bounds.min_position, joint_bounds.max_position) 
                           for joint_bounds in joint_limits]

    # Set the robot to the random joint state
    move_group.set_joint_value_target(random_joint_values)

    # Compute forward kinematics
    success = move_group.go(wait=True)
    if success:
        pose: Pose = move_group.get_current_pose(end_effector_link).pose
        reachable_points.append([pose.position.x, pose.position.y, pose.position.z])

move_group.stop()

# Save points to CSV
csv_file = "/tmp/reachable_workspace.csv"
with open(csv_file, "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y", "z"])
    writer.writerows(reachable_points)

rospy.loginfo(f"Saved {len(reachable_points)} reachable points to {csv_file}")

# Publish as a PointCloud2 message for RViz visualization
cloud_pub = rospy.Publisher("/reachable_workspace_cloud", PointCloud2, queue_size=1)
rospy.sleep(1)  # Ensure publisher is set up before publishing

def create_pointcloud():
    """Create and publish PointCloud2 message."""
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    point_cloud = pc2.create_cloud_xyz32(rospy.Header(frame_id="world"), reachable_points)
    cloud_pub.publish(point_cloud)
    rospy.loginfo("Published reachable workspace point cloud")

# Keep publishing the point cloud
rate = rospy.Rate(0.5)  # 0.5 Hz
while not rospy.is_shutdown():
    create_pointcloud()
    rate.sleep()

moveit_commander.roscpp_shutdown()
