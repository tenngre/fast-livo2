#!/bin/bash

# Lightweight Point Cloud Viewer using PCL command line tools
# This is a simpler alternative to the Open3D viewer, using only PCL tools

# Usage: ./pcl_cli_viewer.sh [topic_name]

# Default topic
DEFAULT_TOPIC="/cloud_registered"

# Set topic from command line argument or use default
TOPIC=${1:-$DEFAULT_TOPIC}

echo "Lightweight PCL Command Line Viewer"
echo "Topic: $TOPIC"
echo "Press Ctrl+C to quit"

echo -e "\nStarting ROS node to convert PointCloud2 to PCD..."

# Create a temporary directory for PCD files
TEMP_DIR=$(mktemp -d)
echo "Temporary directory: $TEMP_DIR"

# Cleanup function
cleanup() {
    echo -e "\nCleaning up..."
    kill $ROS_BAG_PID 2>/dev/null
    rm -rf $TEMP_DIR
    echo "Done."
    exit 0
}

# Trap SIGINT (Ctrl+C) to cleanup
trap cleanup SIGINT

# 1. Start a ROS node that subscribes to the point cloud topic and saves it to a PCD file periodically
rosrun pcl_ros pointcloud_to_pcd input:=/$TOPIC _prefix:=$TEMP_DIR/cloud_ _binary:=false _compressed:=false &
ROS_BAG_PID=$!

# Wait a few seconds for the first PCD file to be created
echo -e "\nWaiting for first PCD file..."
sleep 5

# 2. Continuously display the latest PCD file using pcl_viewer
echo -e "\nStarting pcl_viewer..."
echo "Press H for help, Q to quit"

while true; do
    # Find the latest PCD file
    LATEST_PCD=$(ls -t $TEMP_DIR/cloud_*.pcd 2>/dev/null | head -1)
    if [ -n "$LATEST_PCD" ]; then
        # Display the latest PCD file
        pcl_viewer -bc 0,0,0 -fc 1,1,1 -ps 1 $LATEST_PCD
        # Wait a bit before checking for a new file
        sleep 0.5
    else
        # No PCD file found yet, wait a bit longer
        echo "Waiting for PCD files..."
        sleep 2
    fi
done
