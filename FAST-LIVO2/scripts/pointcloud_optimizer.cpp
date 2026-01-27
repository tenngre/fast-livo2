#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudOptimizer {
public:
    PointCloudOptimizer() {
        // 订阅原始点云话题
        sub_pointcloud = nh.subscribe("/cloud_registered", 1, &PointCloudOptimizer::pointcloudCallback, this);
        
        // 发布优化后的点云话题
        pub_optimized = nh.advertise<sensor_msgs::PointCloud2>("/fast_livo/laser_cloud_optimized", 1);
        
        // 读取参数
        nh.param<double>("voxel_size", voxel_size, 0.2);
        nh.param<int>("publish_rate", publish_rate, 5);
        
        // 计算发布间隔
        publish_interval = 1.0 / publish_rate;
        last_publish_time = ros::Time::now();
        
        ROS_INFO("PointCloud Optimizer initialized with voxel_size: %.2f, publish_rate: %d Hz", 
                 voxel_size, publish_rate);
    }
    
    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // 检查是否到达发布间隔
        ros::Time current_time = ros::Time::now();
        if ((current_time - last_publish_time).toSec() < publish_interval) {
            return;
        }
        
        // 转换为PCL点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);
        
        // 下采样处理
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(voxel_size, voxel_size, voxel_size);
        sor.filter(*cloud_filtered);
        
        // 转换回ROS消息
        sensor_msgs::PointCloud2 optimized_msg;
        pcl::toROSMsg(*cloud_filtered, optimized_msg);
        
        // 保持原始帧ID和时间戳
        optimized_msg.header = msg->header;
        
        // 压缩功能在当前 ROS 版本中不可用，已移除
        
        // 发布优化后的点云
        pub_optimized.publish(optimized_msg);
        
        // 更新最后发布时间
        last_publish_time = current_time;
        
        // 打印信息
        ROS_INFO_THROTTLE(1.0, "Optimized point cloud: %zu -> %zu points (%.1f%% reduction)", 
                         cloud->size(), cloud_filtered->size(), 
                         (1.0 - (double)cloud_filtered->size() / cloud->size()) * 100.0);
    }
    
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_pointcloud;
    ros::Publisher pub_optimized;
    
    double voxel_size;
    int publish_rate;
    double publish_interval;
    ros::Time last_publish_time;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_optimizer");
    PointCloudOptimizer optimizer;
    ros::spin();
    return 0;
}
