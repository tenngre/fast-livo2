/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef GPU_ACCELERATION_H_
#define GPU_ACCELERATION_H_

#include <opencv2/opencv.hpp>
#include <CL/cl.h>
#include <Eigen/Eigen>
#include <vector>
#include <string>

using namespace Eigen;
using namespace std;

class GPUAcceleration {
public:
    GPUAcceleration();
    ~GPUAcceleration();

    // 初始化GPU加速
    bool initialize();

    // 图像补丁提取 GPU加速版
    void getImagePatchGPU(const cv::Mat& img, const Vector2d& pc, float* patch_tmp, int level, int patch_size, int patch_size_total);

    // 仿射变换 GPU加速版
    void warpAffineGPU(const Matrix2d& A_cur_ref, const cv::Mat& img_ref, const Vector2d& px_ref, 
                      int level_ref, int search_level, int pyramid_level, 
                      int halfpatch_size, float* patch, int patch_size, int patch_size_total);

    // 计算NCC GPU加速版
    double calculateNCCGPU(float* ref_patch, float* cur_patch, int patch_size);

    // 检查是否支持GPU加速
    bool isSupported() const { return supported_; }

private:
    bool supported_;
    cl_platform_id platform_;
    cl_device_id device_;
    cl_context context_;
    cl_command_queue queue_;

    // 内核
    cl_program program_;
    cl_kernel kernel_getImagePatch_;
    cl_kernel kernel_warpAffine_;
    cl_kernel kernel_calculateNCC_;

    // 内存对象
    cl_mem img_buffer_;
    cl_mem patch_buffer_;
    cl_mem ref_patch_buffer_;
    cl_mem cur_patch_buffer_;
    cl_mem ncc_result_buffer_;

    // 构建程序
    bool buildProgram();

    // 创建内核
    bool createKernels();

    // 释放资源
    void cleanup();
};

#endif // GPU_ACCELERATION_H_
