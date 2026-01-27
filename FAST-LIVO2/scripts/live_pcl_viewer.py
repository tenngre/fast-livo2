#!/usr/bin/env python3
"""
Lightweight real-time point cloud viewer for FAST-LIVO2
Uses ROS, PCL, and Open3D for minimal memory usage
"""

import os
# Set environment variables for better compatibility with RK3588 OpenGL
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'  # Force software rendering
os.environ['GALLIUM_DRIVER'] = 'softpipe'  # Use softpipe Gallium driver
os.environ['MESA_GL_VERSION_OVERRIDE'] = '3.3'
os.environ['MESA_GLES_VERSION_OVERRIDE'] = '3.1'

import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import argparse
import threading
import time

class LightweightPointCloudViewer:
    def __init__(self, topic, point_size=1.0, background_color=[0, 0, 0]):
        self.topic = topic
        self.point_size = point_size
        self.background_color = background_color
        self.pcd = o3d.geometry.PointCloud()
        self.new_points = False
        self.first_update = True
        self.lock = threading.Lock()
        
        # Initialize ROS node with debug logging
        rospy.init_node('lightweight_pcl_viewer', anonymous=True, log_level=rospy.DEBUG)
        
        # Subscribe to point cloud topic
        rospy.Subscriber(self.topic, PointCloud2, self.point_cloud_callback)
        
        # Initialize Open3D visualizer with smaller window size for RK3588
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name=f"Point Cloud Viewer: {self.topic}", width=800, height=600)
        self.vis.add_geometry(self.pcd)
        
        # Set visualizer properties for better visibility on RK3588
        render_option = self.vis.get_render_option()
        render_option.point_size = self.point_size
        render_option.background_color = np.asarray(self.background_color)
        render_option.point_show_normal = False
        render_option.line_width = 1.0
        render_option.mesh_show_wireframe = False
        render_option.mesh_show_back_face = False
        
        # Set default view control
        self.view_control = self.vis.get_view_control()
        
        print(f"Lightweight PCL Viewer started, subscribing to {self.topic}")
        print("Controls:")
        print("  - Mouse: Rotate, Zoom, Pan")
        print("  - Q: Quit")
        print("  - R: Reset view")
    
    def point_cloud_callback(self, msg):
        """Callback function for point cloud messages"""
        # Debug: Print message received
        rospy.logdebug(f"Received point cloud message with {msg.width} points")
        
        # Convert ROS PointCloud2 to numpy array
        points_list = []
        for data in pc2.read_points(msg, skip_nans=True):
            # Extract x, y, z coordinates (ignore intensity/colors for simplicity)
            points_list.append([data[0], data[1], data[2]])
        
        if points_list:
            rospy.logdebug(f"Converted {len(points_list)} valid points")
            with self.lock:
                # Update point cloud data
                self.pcd.points = o3d.utility.Vector3dVector(np.array(points_list))
                
                # Set point colors to red for better visibility (white on black background)
                colors = np.ones((len(points_list), 3)) * [1.0, 0.0, 0.0]  # Red color
                self.pcd.colors = o3d.utility.Vector3dVector(colors)
                
                # Ensure the point cloud is properly initialized
                if not self.pcd.has_points():
                    rospy.logwarn("Point cloud has no points after update")
                self.new_points = True
        else:
            rospy.logwarn("No valid points in received message")
    
    def run(self):
        """Main visualization loop"""
        while not rospy.is_shutdown():
            # Update geometry if new points are available
            with self.lock:
                if self.new_points:
                    self.vis.update_geometry(self.pcd)
                    
                    # Auto-adjust view on first update to ensure all points are visible
                    if self.first_update:
                        print("Auto-adjusting view to fit point cloud...")
                        self.vis.reset_view_point(True)  # Reset view to fit geometry
                        self.first_update = False
                    
                    self.new_points = False
            
            # Check if window is closed
            if not self.vis.poll_events():
                break
            
            # Update visualization
            self.vis.update_renderer()
            
            # Small sleep to reduce CPU usage
            time.sleep(0.01)
        
        # Cleanup
        self.vis.destroy_window()

def main():
    parser = argparse.ArgumentParser(description='Lightweight real-time point cloud viewer for FAST-LIVO2')
    parser.add_argument('-t', '--topic', type=str, default='/cloud_registered',
                        help='ROS topic to subscribe to (default: /cloud_registered)')
    parser.add_argument('-s', '--point-size', type=float, default=1.0,
                        help='Point size for visualization (default: 1.0)')
    parser.add_argument('-b', '--background', type=str, default='0,0,0',
                        help='Background color in RGB (default: 0,0,0)')
    
    args = parser.parse_args()
    
    # Parse background color
    background_color = [float(c) / 255.0 for c in args.background.split(',')]
    
    # Create and run viewer
    viewer = LightweightPointCloudViewer(
        topic=args.topic,
        point_size=args.point_size,
        background_color=background_color
    )
    viewer.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
