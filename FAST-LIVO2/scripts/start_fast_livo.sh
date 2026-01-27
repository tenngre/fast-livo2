#!/bin/bash

# 启动脚本：依次启动FAST-LIVO2及其依赖组件
# 1. roslaunch livox_ros_driver2 msg_MID360.launch
# 2. roslaunch usb_cam usb_cam.launch
# 3. roslaunch fast_livo mapping_mid360.launch
# 4. python3 /home/firefly/catkin_ws/src/rpg_vikit/FAST-LIVO2/scripts/opengl_pointcloud_viewer.py

echo "Starting FAST-LIVO2 system..."

# 切换到catkin_ws目录
cd /home/firefly/catkin_ws

# 1. 启动Livox雷达驱动
echo "Starting Livox ROS Driver 2..."
roslaunch livox_ros_driver2 msg_MID360.launch &
LIVOX_PID=$!

# 等待2秒让雷达驱动初始化
sleep 2

# 2. 启动USB相机
echo "Starting USB Camera..."
roslaunch usb_cam usb_cam.launch &
USB_CAM_PID=$!

# 等待2秒让相机初始化
sleep 2

# 3. 启动FAST-LIVO2映射
echo "Starting FAST-LIVO2 mapping..."
roslaunch fast_livo mapping_mid360.launch &
FAST_LIVO_PID=$!

# 等待3秒让FAST-LIVO2初始化
sleep 3

# 4. 启动PointCloud Viewer
echo "Starting PointCloud Viewer..."
python3 /home/firefly/catkin_ws/src/rpg_vikit/FAST-LIVO2/scripts/pointcloud_viewer.py &
OPENGL_PID=$!

echo "All components started successfully!"
echo "Process IDs:"
echo "Livox: $LIVOX_PID"
echo "USB Camera: $USB_CAM_PID"
echo "FAST-LIVO2: $FAST_LIVO_PID"
echo "OpenGL Viewer: $OPENGL_PID"
echo ""
echo "To stop all components, run: kill $LIVOX_PID $USB_CAM_PID $FAST_LIVO_PID $OPENGL_PID"
