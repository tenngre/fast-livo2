#!/bin/bash

# 终止脚本：停止FAST-LIVO2及其依赖组件
# 1. roslaunch livox_ros_driver2 msg_MID360.launch
# 2. roslaunch usb_cam usb_cam.launch
# 3. roslaunch fast_livo mapping_mid360.launch
# 4. python3 /home/firefly/catkin_ws/src/rpg_vikit/FAST-LIVO2/scripts/opengl_pointcloud_viewer.py

echo "Stopping FAST-LIVO2 system..."

# 1. 停止OpenGL点云查看器
echo "Stopping OpenGL PointCloud Viewer..."
pkill -f "opengl_pointcloud_viewer.py"

# 2. 停止FAST-LIVO2映射
echo "Stopping FAST-LIVO2 mapping..."
pkill -f "roslaunch fast_livo mapping_mid360.launch"

# 3. 停止USB相机
echo "Stopping USB Camera..."
pkill -f "roslaunch usb_cam usb_cam.launch"

# 4. 停止Livox雷达驱动
echo "Stopping Livox ROS Driver 2..."
pkill -f "roslaunch livox_ros_driver2 msg_MID360.launch"

# 等待所有进程终止
sleep 2

echo "All components stopped successfully!"

# 验证所有相关进程是否已终止
echo "Checking for remaining processes..."
LIVOX_REMAINING=$(ps aux | grep "livox_ros_driver2" | grep -v grep | wc -l)
USB_CAM_REMAINING=$(ps aux | grep "usb_cam.launch" | grep -v grep | wc -l)
FAST_LIVO_REMAINING=$(ps aux | grep "fast_livo" | grep -v grep | wc -l)
OPENGL_REMAINING=$(ps aux | grep "opengl_pointcloud_viewer.py" | grep -v grep | wc -l)

if [ $LIVOX_REMAINING -eq 0 ] && [ $USB_CAM_REMAINING -eq 0 ] && [ $FAST_LIVO_REMAINING -eq 0 ] && [ $OPENGL_REMAINING -eq 0 ]; then
    echo "No remaining processes found."
else
    echo "Warning: Some processes may still be running:"
    if [ $LIVOX_REMAINING -gt 0 ]; then
        echo "- Livox ROS Driver 2: $LIVOX_REMAINING processes"
    fi
    if [ $USB_CAM_REMAINING -gt 0 ]; then
        echo "- USB Camera: $USB_CAM_REMAINING processes"
    fi
    if [ $FAST_LIVO_REMAINING -gt 0 ]; then
        echo "- FAST-LIVO2: $FAST_LIVO_REMAINING processes"
    fi
    if [ $OPENGL_REMAINING -gt 0 ]; then
        echo "- OpenGL Viewer: $OPENGL_REMAINING processes"
    fi
    echo "You may need to kill them manually."
fi
