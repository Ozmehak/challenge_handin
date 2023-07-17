#! /usr/bin/env python3
import os
import shutil
import subprocess

# Define paths to patch files and target directory
controller_patch_path = os.path.join(os.getcwd(), 'controller.patch')
rviz_patch_path = os.path.join(os.getcwd(), 'rviz.patch')
ros2_control_demos_path = os.path.abspath(os.path.join(os.getcwd(), '../../ros2_control_demos'))

# Copy patch files to target directory
shutil.copy(controller_patch_path, ros2_control_demos_path)
shutil.copy(rviz_patch_path, ros2_control_demos_path)

# Apply patches using Git
subprocess.run(['git', 'apply', 'controller.patch'], cwd=ros2_control_demos_path, check=True)
subprocess.run(['git', 'apply', 'rviz.patch'], cwd=ros2_control_demos_path, check=True)
