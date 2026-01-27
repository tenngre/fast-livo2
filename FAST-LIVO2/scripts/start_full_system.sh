#!/bin/bash

# 完整的FAST-LIVO2启动脚本，包括轻量级点云查看器
# 使用方法：bash start_full_system.sh

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== FAST-LIVO2完整启动脚本 ===${NC}"
echo "这将启动ROS master、FAST-LIVO2系统和轻量级点云查看器"
echo -e "按 ${YELLOW}Ctrl+C${NC} 停止所有进程"

# 检查是否已source setup.bash
if [ -z "$ROS_PACKAGE_PATH" ]; then
    echo -e "${YELLOW}警告：未找到ROS环境变量，正在source setup.bash...${NC}"
    source /home/firefly/catkin_ws/devel/setup.bash
fi

# 1. 启动ROS master
if ! pgrep -x "roscore" > /dev/null; then
    echo -e "${GREEN}[1/4] 启动ROS master...${NC}"
    roscore &
    ROSCORE_PID=$!
    sleep 5  # 等待roscore启动
else
    echo -e "${GREEN}[1/4] ROS master已在运行${NC}"
fi

# 2. 启动LiDAR驱动
echo -e "${GREEN}[2/4] 启动LiDAR驱动...${NC}"
roslaunch livox_ros_driver2 msg_MID360.launch &
LIVOX_PID=$!
sleep 3

# 3. 启动USB相机
echo -e "${GREEN}[3/4] 启动USB相机...${NC}"
roslaunch usb_cam usb_cam.launch &
USB_CAM_PID=$!
sleep 3

# 4. 启动FAST-LIVO2建图
echo -e "${GREEN}[4/4] 启动FAST-LIVO2建图...${NC}"
roslaunch fast_livo mapping_mid360.launch &
FAST_LIVO_PID=$!
sleep 5

# 5. 启动轻量级点云查看器
echo -e "${GREEN}启动轻量级点云查看器...${NC}"
python3 /home/firefly/catkin_ws/src/rpg_vikit/FAST-LIVO2/scripts/live_pcl_viewer.py &
VIEWER_PID=$!

# 显示状态
echo -e "${GREEN}=== 系统已启动 ===${NC}"
echo "所有进程正在运行中..."
echo "- ROS master: $(pgrep -x "roscore" > /dev/null && echo "运行中" || echo "未运行")"
echo "- LiDAR驱动: $(ps -p $LIVOX_PID > /dev/null && echo "运行中" || echo "未运行")"
echo "- USB相机: $(ps -p $USB_CAM_PID > /dev/null && echo "运行中" || echo "未运行")"
echo "- FAST-LIVO2: $(ps -p $FAST_LIVO_PID > /dev/null && echo "运行中" || echo "未运行")"
echo "- 点云查看器: $(ps -p $VIEWER_PID > /dev/null && echo "运行中" || echo "未运行")"
echo -e "${YELLOW}按Ctrl+C停止所有进程${NC}"

# 检查点云话题是否存在
echo -e "${GREEN}\n检查点云话题...${NC}"
sleep 3
rostopic list | grep cloud || echo -e "${YELLOW}警告：未找到点云话题，可能需要等待更长时间${NC}"

# 等待用户输入Ctrl+C
trap "echo -e '\n${GREEN}停止所有进程...${NC}'; kill $ROSCORE_PID $LIVOX_PID $USB_CAM_PID $FAST_LIVO_PID $VIEWER_PID 2>/dev/null; echo '完成'" SIGINT
wait