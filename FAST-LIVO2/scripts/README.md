# Lightweight Point Cloud Viewer for FAST-LIVO2

This is a lightweight real-time point cloud viewer for FAST-LIVO2, designed to be more memory-efficient than RVIZ.

## Features

- **Low memory usage**: Uses only essential components for point cloud visualization
- **Real-time**: Efficiently updates point clouds as they arrive
- **Easy to use**: Simple command-line interface with minimal dependencies
- **Interactive**: Mouse controls for rotating, zooming, and panning
- **Customizable**: Adjustable point size and background color

## Dependencies

- Python 3
- ROS Noetic
- Open3D
- numpy
- sensor_msgs (ROS package)

## Installation

1. Ensure you have ROS Noetic installed
2. Install required Python packages:
   ```bash
   pip3 install open3d numpy
   ```
3. Build your FAST-LIVO2 workspace:
   ```bash
   cd /home/firefly/catkin_ws
   catkin_make
   ```

## Usage

### Basic Usage

1. First, start your FAST-LIVO2 system as usual:
   ```bash
   # Terminal 1: Start LiDAR driver
   roslaunch livox_ros_driver2 msg_MID360.launch
   
   # Terminal 2: Start USB camera
   roslaunch usb_cam usb_cam.launch
   
   # Terminal 3: Start FAST-LIVO2 mapping
   roslaunch fast_livo mapping_mid360.launch
   ```

2. In a new terminal, run the lightweight point cloud viewer:
   ```bash
   source /home/firefly/catkin_ws/devel/setup.bash
   python3 /home/firefly/catkin_ws/src/rpg_vikit/FAST-LIVO2/scripts/live_pcl_viewer.py
   ```

### Advanced Options

- **Change point cloud topic**: Use the `-t` or `--topic` parameter
  ```bash
  python3 live_pcl_viewer.py -t /Laser_map
  ```

- **Adjust point size**: Use the `-s` or `--point-size` parameter
  ```bash
  python3 live_pcl_viewer.py -s 2.0
  ```

- **Change background color**: Use the `-b` or `--background` parameter with RGB values (0-255)
  ```bash
  python3 live_pcl_viewer.py -b 255,255,255  # White background
  ```

### Available Point Cloud Topics from FAST-LIVO2

- `/cloud_registered`: Registered point cloud (default)
- `/Laser_map`: Full laser map
- `/cloud_effected`: Effected point cloud
- `/dyn_obj`: Dynamic objects
- `/dyn_obj_removed`: Point cloud with dynamic objects removed

## Controls

- **Mouse**: Rotate, Zoom, Pan
- **Q**: Quit
- **R**: Reset view

## Performance Tips

1. Use a smaller point size for better performance
2. Subscribe to filtered point cloud topics when possible
3. Close other memory-intensive applications
4. Consider reducing the point cloud density in FAST-LIVO2 configuration if needed

## Troubleshooting

### No Point Cloud Displayed

- Check if the ROS master is running
- Verify the correct topic name is being used
- Ensure FAST-LIVO2 is publishing point clouds
- Check the ROS network configuration if running across multiple machines

### Performance Issues

- Reduce point size
- Try a different point cloud topic with fewer points
- Close other applications consuming memory or CPU

## Comparison with RVIZ

| Feature | Lightweight Viewer | RVIZ |
|---------|-------------------|------|
| Memory Usage | Low | High |
| CPU Usage | Low | Medium-High |
| Startup Time | Fast | Slow |
| Customization | Basic | Extensive |
| Ease of Use | Very Easy | Moderate |

## License

MIT License
