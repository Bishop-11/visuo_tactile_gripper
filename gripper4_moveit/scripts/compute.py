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
# CHANGE GROUP NAME
group_name = "left_arm"  # Change this to your MoveIt!
move_group = moveit_commander.MoveGroupCommander(group_name)

# Get the end effector link name
end_effector_link = move_group.get_end_effector_link()
if not end_effector_link:
    rospy.logerr("No end effector link found!")
    exit()

rospy.loginfo(f"Using End Effector Link: {end_effector_link}")

# Retrieve joint limits
#joint_limits = move_group.get_joints()
#joint_limits = [move_group.get_joint_limits(joint) for joint in joint_limits if move_group.has_joint(joint)]

joint_names = move_group.get_joints()
#joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_5', 'joint_7', 'joint_9', 'joint_11', 'joint_13', 'joint_15']
rospy.loginfo(f"Joints : {str(joint_names)}")

# Get joint limits using ROS parameter server
joint_limits = []
for joint in joint_names:
    #min_limit = rospy.get_param(f"/robot_moveit3/config/joint_limits/{joint}/min_position", None)
    #max_limit = rospy.get_param(f"/robot_moveit3/config/joint_limits/{joint}/max_position", None)
    min_limit = -1.57
    max_limit = 1.57

    if min_limit is not None and max_limit is not None:
        joint_limits.append({"name": joint, "min": min_limit, "max": max_limit})

rospy.loginfo(f"Joint Limits : {str(joint_limits)}")

for joint in joint_limits:
    rospy.loginfo(f"Joint {joint['name']}: min={joint['min']}, max={joint['max']}")

# SET NUMBER OF SAMPLES
num_samples = 5000  # Increase for better resolution
reachable_points = []

# SET INVALID (PASSIVE) JOINTS
invalid_joint_names = [
    'joint_0',
    'joint_1',
    'joint_3',
    'joint_7',
    'joint_9',
    'joint_11',
    'joint_15',
    'joint_19',
    'joint_21',
    'joint_23',
    'joint_27',
    'joint_29',
    'joint_31']

# Randomly sample joint configurations and compute end-effector positions
rospy.loginfo("Sampling joint space and computing reachable workspace...")
for sample_num in range(num_samples):
    rospy.loginfo(f"Sample Number : {sample_num}")
    
    random_joint_values = []
    for joint in joint_limits:
        #random_joint_values = [np.random.uniform(joint_bounds.min_position, joint_bounds.max_position) for joint_bounds in joint_limits]    
        #random_joint_values = [max(joint["min"], min(joint["max"], np.random.uniform(joint["min"], joint["max"]))) for joint in joint_limits]
        if joint['name'] in invalid_joint_names:
            #random_joint_values.append(0.00000000)
            continue
        else:
            random_joint_values.append(max(joint["min"], min(joint["max"], np.random.uniform(joint["min"], joint["max"]))))

    if sample_num%(num_samples/10) == 0:
        rospy.loginfo(f"random_joint_values : {str(random_joint_values)}")

    try:
        # Try setting the joint values as the target
        move_group.set_joint_value_target(random_joint_values)
        success = move_group.go(wait=True)

        if success:
            rospy.loginfo("Success")
            pose = move_group.get_current_pose(end_effector_link).pose
            reachable_points.append([pose.position.x, pose.position.y, pose.position.z])

    except moveit_commander.MoveItCommanderException as e:
        rospy.loginfo("Failed")
        #rospy.logwarn(f"Skipping invalid joint target: {random_joint_values} - {e}")
        continue  # Skip this iteration and try again


    if sample_num%(num_samples/10) == 0:
        # Save points to CSV
        csv_file = "/home/bishop/Documents/ROS_Workspaces/gripper_ws/src/visuo_tactile_gripper/gripper4_moveit/saved_points/reachable_workspace1.csv"
        with open(csv_file, "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "z"])
            writer.writerows(reachable_points)

move_group.stop()

# Save points to CSV
csv_file = "/home/bishop/Documents/ROS_Workspaces/gripper_ws/src/visuo_tactile_gripper/gripper4_moveit/saved_points/reachable_workspace1.csv"
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