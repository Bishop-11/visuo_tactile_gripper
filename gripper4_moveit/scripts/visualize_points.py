#!/usr/bin/env python3

import csv
import numpy as np
import matplotlib.pyplot as plt
import argparse
from mpl_toolkits.mplot3d import Axes3D  # Ensure 3D plotting support


def visualize_xyz(csv_file):
    x, y, z = [], [], []

    # Read CSV file manually
    try:
        with open(csv_file, "r") as f:
            reader = csv.reader(f)
            next(reader)  # Skip header row

            for row in reader:
                try:
                    x.append(float(row[0]))
                    y.append(float(row[1]))
                    z.append(float(row[2]))
                except ValueError:
                    print(f"Skipping invalid row: {row}")

    except FileNotFoundError:
        print(f"Error: File '{csv_file}' not found!")
        return
    except Exception as e:
        print(f"Error reading file: {e}")
        return

    if not x:
        print("Error: No valid data found in the CSV file!")
        return

    # Plot 3D scatter plot
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    sc = ax.scatter(x, y, z, c=z, cmap="jet", marker="o", alpha=0.6)

    # Labels and title
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    ax.set_title("3D Visualization of Reachable Workspace")
    fig.colorbar(sc, ax=ax, label="Z value")

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize XYZ coordinates from a CSV file.")
    parser.add_argument("csv_file", nargs="?", 
                        default="/home/bishop/Documents/ROS_Workspaces/gripper_ws/src/visuo_tactile_gripper/robot_moveit3/saved_points/reachable_workspace2.csv",
                        help="Path to CSV file (default: /home/bishop/Documents/ROS_Workspaces/gripper_ws/src/visuo_tactile_gripper/robot_moveit3/saved_points/reachable_workspace.csv")
    args = parser.parse_args()

    visualize_xyz(args.csv_file)