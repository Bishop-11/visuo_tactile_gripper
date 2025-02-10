import rospy
import moveit_commander
from moveit_commander.robot_trajectory import RobotTrajectory
from moveit_commander.motion_planners import MoveGroupCommander
from geometry_msgs.msg import Pose
import math

def get_end_effector_range():
    # Initialize moveit_commander and ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('end_effector_range_calculation', anonymous=True)

    # Initialize the MoveGroupCommander for your robot's planning group
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("your_robot_arm")

    # Get the current end effector pose (position and orientation)
    current_pose = group.get_current_pose().pose
    print(f"Current Pose: {current_pose}")

    # Set a range of joint configurations to test the robot's reach
    # For example, generate a grid of joint values within the joint limits
    joint_limits = group.get_active_joints()
    reachable_points = []
    for joint_config in joint_limits:
        group.set_joint_value_target(joint_config)  # Set a new joint configuration
        plan = group.plan()  # Plan the motion to the new joint configuration
        trajectory = RobotTrajectory()
        trajectory.set_trajectory_from_plan(plan)
        
        # Get the pose of the end effector for this configuration
        new_pose = group.get_current_pose().pose
        reachable_points.append(new_pose.position)
    
    # Calculate the farthest distance from the origin (or any reference point)
    max_distance = 0
    for point in reachable_points:
        distance = math.sqrt(point.x**2 + point.y**2 + point.z**2)
        max_distance = max(max_distance, distance)

    print(f"Maximum Reachable Distance: {max_distance} meters")
    moveit_commander.roscpp_shutdown()
    
if __name__ == "__main__":
    get_end_effector_range()
