#!/usr/bin/env python3
"""
Test script to publish a simple point cloud for testing the lightweight viewer
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

# Define point cloud fields
FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]

def publish_test_point_cloud():
    """Publish a simple test point cloud"""
    rospy.init_node('test_point_cloud_publisher')
    pub = rospy.Publisher('/cloud_registered', PointCloud2, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    
    print("Test Point Cloud Publisher started")
    print("Publishing simple cube point cloud to /cloud_registered")
    
    # Create a simple cube point cloud
    points = []
    for x in np.linspace(-1, 1, 10):
        for y in np.linspace(-1, 1, 10):
            for z in np.linspace(-1, 1, 10):
                points.append([x, y, z, 1.0])  # x, y, z, intensity
    
    # Convert to numpy array
    points = np.array(points, dtype=np.float32)
    
    while not rospy.is_shutdown():
        # Create PointCloud2 message
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'camera_init'
        
        # Create point cloud message
        cloud_msg = pc2.create_cloud(header, FIELDS, points)
        
        # Publish message
        pub.publish(cloud_msg)
        
        # Sleep
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_test_point_cloud()
    except rospy.ROSInterruptException:
        pass
