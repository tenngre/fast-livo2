#!/usr/bin/env python3

import rospy
import struct
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class PointCloudColorAnalyzer:
    def __init__(self):
        rospy.Subscriber('/cloud_registered', PointCloud2, self.point_cloud_callback)
        self.count = 0
        self.max_points = 20
    
    def point_cloud_callback(self, msg):
        if self.count >= self.max_points:
            return
        
        print(f"\n=== Point Cloud #{self.count} ===")
        print(f"Height: {msg.height}, Width: {msg.width}")
        print(f"Point step: {msg.point_step}, Row step: {msg.row_step}")
        
        # 打印字段信息
        print("Fields:")
        for field in msg.fields:
            print(f"  - {field.name}: offset={field.offset}, datatype={field.datatype}, count={field.count}")
        
        # 读取前10个点的信息
        print("\nFirst 10 points:")
        point_count = 0
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True):
            if point_count >= 10:
                break
            
            x, y, z, rgb = p
            print(f"Point {point_count}:")
            print(f"  Position: ({x:.3f}, {y:.3f}, {z:.3f})")
            print(f"  RGB value: {rgb}")
            
            # 尝试不同的颜色解析方法
            try:
                # 方法1：直接将float转换为int
                rgb_int = struct.unpack('I', struct.pack('f', rgb))[0]
                r = ((rgb_int >> 16) & 0xff) / 255.0
                g = ((rgb_int >> 8) & 0xff) / 255.0
                b = (rgb_int & 0xff) / 255.0
                print(f"  Method 1 - RGB: ({r:.3f}, {g:.3f}, {b:.3f})")
            except Exception as e:
                print(f"  Method 1 failed: {e}")
            
            try:
                # 方法2：假设rgb是0-1之间的浮点数
                if 0.0 <= rgb <= 1.0:
                    print(f"  Method 2 - Gray: {rgb:.3f}")
                else:
                    print(f"  Method 2 - Out of range: {rgb}")
            except Exception as e:
                print(f"  Method 2 failed: {e}")
            
            try:
                # 方法3：直接使用rgb值作为颜色分量
                r = g = b = rgb
                print(f"  Method 3 - RGB: ({r:.3f}, {g:.3f}, {b:.3f})")
            except Exception as e:
                print(f"  Method 3 failed: {e}")
            
            point_count += 1
        
        self.count += 1

if __name__ == '__main__':
    rospy.init_node('point_cloud_color_analyzer')
    analyzer = PointCloudColorAnalyzer()
    rospy.spin()
