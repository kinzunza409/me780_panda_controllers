#!/bin/bash
XACRO_PATH=/opt/ros/humble/share/franka_description/robots/fer/fer.urdf.xacro
OUTPUT_PATH=/workspace/assets/models/custom/panda_arm.urdf

ros2 run xacro xacro "$XACRO_PATH" hand:=false > "$OUTPUT_PATH"

echo "URDF generated at $OUTPUT_PATH"