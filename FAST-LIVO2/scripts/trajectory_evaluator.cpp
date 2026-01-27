#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

class TrajectoryEvaluator {
public:
    TrajectoryEvaluator() {
        // 订阅FAST-LIVO2发布的里程计话题
        sub_odom = nh.subscribe("/aft_mapped_to_init", 100, &TrajectoryEvaluator::odomCallback, this);
        
        // 发布评估结果路径
        pub_eval_path = nh.advertise<nav_msgs::Path>("/fast_livo/trajectory_evaluated", 10);
        
        // 读取参数
        nh.param<std::string>("output_path", output_path, "/home/firefly/catkin_ws/src/rpg_vikit/FAST-LIVO2/Log/result/trajectory.txt");
        nh.param<bool>("save_trajectory", save_trajectory, true);
        
        // 打开输出文件
        if (save_trajectory) {
            outfile.open(output_path);
            if (!outfile.is_open()) {
                ROS_ERROR("Failed to open output file: %s", output_path.c_str());
            } else {
                ROS_INFO("Saving trajectory to: %s", output_path.c_str());
            }
        }
        
        // 初始化路径消息
        eval_path.header.frame_id = "camera_init";
    }
    
    ~TrajectoryEvaluator() {
        if (outfile.is_open()) {
            outfile.close();
        }
    }
    
    void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        // 提取时间戳、位置和姿态
        double timestamp = msg->header.stamp.toSec();
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        
        // 保存轨迹到TUM格式文件
        if (outfile.is_open()) {
            outfile << std::fixed << std::setprecision(9) << timestamp << " " 
                    << std::fixed << std::setprecision(6) << x << " " << y << " " << z << " " 
                    << qx << " " << qy << " " << qz << " " << qw << std::endl;
        }
        
        // 更新评估路径
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose = msg->pose.pose;
        eval_path.poses.push_back(pose_stamped);
        eval_path.header.stamp = msg->header.stamp;
        pub_eval_path.publish(eval_path);
    }
    
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_odom;
    ros::Publisher pub_eval_path;
    
    std::string output_path;
    bool save_trajectory;
    std::ofstream outfile;
    
    nav_msgs::Path eval_path;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_evaluator");
    TrajectoryEvaluator evaluator;
    ros::spin();
    return 0;
}
