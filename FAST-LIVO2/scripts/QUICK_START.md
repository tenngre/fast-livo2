# 轻量级点云查看器快速入门

## 问题诊断

如果您执行 `python3 live_pcl_viewer.py` 后没有看到点云显示，请按照以下步骤进行诊断：

### 1. 检查ROS话题是否存在

```bash
# 查看所有ROS话题
rostopic list

# 特别检查点云相关话题
rostopic list | grep cloud
```

您应该能看到类似以下的点云话题：
- `/cloud_registered`
- `/Laser_map`
- `/cloud_effected`

### 2. 检查点云话题是否有数据

```bash
# 查看点云话题的发布者和订阅者
rostopic info /cloud_registered

# 检查点云数据是否流动
rostopic hz /cloud_registered
```

### 3. 检查点云查看器是否正在接收数据

运行点云查看器后，您应该能在终端看到类似以下的调试信息：
```
[DEBUG] [1769424384.368773]: Received point cloud message with 1000 points
[DEBUG] [1769424384.368773]: Converted 1000 valid points
```

## 快速测试方法

### 1. 使用测试点云发布者

我已经创建了一个测试点云发布者，用于发布一个简单的立方体点云。您可以使用它来测试点云查看器是否能正常工作：

```bash
# 启动测试点云发布者
source /home/firefly/catkin_ws/devel/setup.bash
python3 /home/firefly/catkin_ws/src/rpg_vikit/FAST-LIVO2/scripts/test_publisher.py
```

然后在另一个终端启动点云查看器：

```bash
python3 /home/firefly/catkin_ws/src/rpg_vikit/FAST-LIVO2/scripts/live_pcl_viewer.py
```

### 2. 测试不同的点云话题

您可以尝试查看不同的点云话题，以确定哪个话题包含您需要的点云数据：

```bash
# 查看完整激光地图
python3 live_pcl_viewer.py -t /Laser_map

# 查看注册点云（默认）
python3 live_pcl_viewer.py -t /cloud_registered

# 查看移除动态物体后的点云
python3 live_pcl_viewer.py -t /dyn_obj_removed
```

## 常见问题解决方案

### 问题1：窗口启动了，但没有点云显示

**可能原因**：
- 点云话题没有数据
- 点云数据范围太大或太小，超出了可视范围
- 点云颜色与背景颜色相近，导致不可见

**解决方案**：
- 检查点云话题是否有数据流动：`rostopic hz /cloud_registered`
- 尝试使用不同的点云话题
- 尝试调整点云查看器的背景颜色和点大小：
  ```bash
  # 使用白色背景和更大的点
  python3 live_pcl_viewer.py -b 255,255,255 -s 2.0
  ```

### 问题2：点云查看器崩溃或窗口无法打开

**可能原因**：
- OpenGL驱动问题（特别是在RK3588等ARM平台）
- 内存不足
- 图形硬件不支持

**解决方案**：
- 确保您使用的是软件渲染（脚本中已设置）
- 关闭其他不必要的应用程序，释放内存
- 尝试使用PCL命令行查看器作为备选方案：
  ```bash
  /home/firefly/catkin_ws/src/rpg_vikit/FAST-LIVO2/scripts/pcl_cli_viewer.sh
  ```

### 问题3：点云显示不流畅

**可能原因**：
- 点云数据量太大
- CPU或GPU资源不足

**解决方案**：
- 尝试使用数据量较小的点云话题
- 减小点大小：`python3 live_pcl_viewer.py -s 0.5`
- 增加sleep时间，降低更新频率（修改脚本中的time.sleep值）

## 性能优化建议

1. **降低点大小**：使用 `-s` 参数减小点大小，例如 `-s 0.5`
2. **减少点云密度**：在FAST-LIVO2配置中增加下采样参数
3. **使用较小的窗口大小**：修改脚本中的 `width` 和 `height` 参数
4. **增加sleep时间**：修改脚本中的 `time.sleep(0.01)` 为 `time.sleep(0.05)`
5. **关闭不必要的点云话题**：只启动必要的点云话题

## 备选方案：使用PCL命令行查看器

如果Open3D查看器仍然无法正常工作，您可以尝试使用PCL命令行查看器：

```bash
# 运行PCL命令行查看器
bash /home/firefly/catkin_ws/src/rpg_vikit/FAST-LIVO2/scripts/pcl_cli_viewer.sh

# 查看特定话题
bash /home/firefly/catkin_ws/src/rpg_vikit/FAST-LIVO2/scripts/pcl_cli_viewer.sh /Laser_map
```

## 完整启动脚本

我已经创建了一个完整的启动脚本，用于一键启动所有必需的进程：

```bash
# 运行完整启动脚本
bash /home/firefly/catkin_ws/src/rpg_vikit/FAST-LIVO2/scripts/start_full_system.sh
```

这个脚本将启动：
1. ROS master
2. LiDAR驱动
3. USB相机
4. FAST-LIVO2建图
5. 轻量级点云查看器

## 联系支持

如果您仍然遇到问题，请检查终端输出的错误信息，并尝试根据错误信息进行调试。您也可以尝试查看ROS日志，获取更多调试信息：

```bash
# 查看ROS日志
rqt_console
```

或者查看点云查看器的详细日志：

```bash
# 使用更详细的日志级别运行点云查看器
python3 live_pcl_viewer.py --ros-args -l debug
```
