/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "gpu_acceleration.h"
#include <iostream>
#include <fstream>

GPUAcceleration::GPUAcceleration() : supported_(false) {
    platform_ = nullptr;
    device_ = nullptr;
    context_ = nullptr;
    queue_ = nullptr;
    program_ = nullptr;
    kernel_getImagePatch_ = nullptr;
    kernel_warpAffine_ = nullptr;
    kernel_calculateNCC_ = nullptr;
    img_buffer_ = nullptr;
    patch_buffer_ = nullptr;
    ref_patch_buffer_ = nullptr;
    cur_patch_buffer_ = nullptr;
    ncc_result_buffer_ = nullptr;
}

GPUAcceleration::~GPUAcceleration() {
    cleanup();
}

bool GPUAcceleration::initialize() {
    // 获取平台
    cl_int err = clGetPlatformIDs(1, &platform_, NULL);
    if (err != CL_SUCCESS) {
        std::cerr << "Error: Failed to get platform ID" << std::endl;
        return false;
    }

    // 获取设备
    err = clGetDeviceIDs(platform_, CL_DEVICE_TYPE_GPU, 1, &device_, NULL);
    if (err != CL_SUCCESS) {
        std::cerr << "Error: Failed to get device ID" << std::endl;
        return false;
    }

    // 创建上下文
    context_ = clCreateContext(NULL, 1, &device_, NULL, NULL, &err);
    if (err != CL_SUCCESS) {
        std::cerr << "Error: Failed to create context" << std::endl;
        return false;
    }

    // 创建命令队列
    queue_ = clCreateCommandQueue(context_, device_, 0, &err);
    if (err != CL_SUCCESS) {
        std::cerr << "Error: Failed to create command queue" << std::endl;
        return false;
    }

    // 构建程序
    if (!buildProgram()) {
        return false;
    }

    // 创建内核
    if (!createKernels()) {
        return false;
    }

    supported_ = true;
    std::cout << "GPU acceleration initialized successfully" << std::endl;
    return true;
}

bool GPUAcceleration::buildProgram() {
    // OpenCL 内核代码
    const char* kernel_code = R"(
        __kernel void getImagePatch(
            __global const uchar* img,
            __global float* patch,
            float u_ref,
            float v_ref,
            int level,
            int patch_size,
            int patch_size_total,
            int width,
            int height
        ) {
            int idx = get_global_id(0);
            if (idx >= patch_size_total) return;

            int x = idx % patch_size;
            int y = idx / patch_size;

            int scale = 1 << level;
            int u_ref_i = (int)(u_ref / scale) * scale;
            int v_ref_i = (int)(v_ref / scale) * scale;
            float subpix_u_ref = (u_ref - u_ref_i) / scale;
            float subpix_v_ref = (v_ref - v_ref_i) / scale;

            float w_ref_tl = (1.0f - subpix_u_ref) * (1.0f - subpix_v_ref);
            float w_ref_tr = subpix_u_ref * (1.0f - subpix_v_ref);
            float w_ref_bl = (1.0f - subpix_u_ref) * subpix_v_ref;
            float w_ref_br = subpix_u_ref * subpix_v_ref;

            int img_x = u_ref_i - (patch_size / 2) * scale + y * scale;
            int img_y = v_ref_i - (patch_size / 2) * scale + x * scale;

            if (img_x >= 0 && img_x < width && img_y >= 0 && img_y < height) {
                int img_idx = img_y * width + img_x;
                patch[idx] = w_ref_tl * img[img_idx] + 
                            w_ref_tr * img[img_idx + scale] + 
                            w_ref_bl * img[img_idx + scale * width] + 
                            w_ref_br * img[img_idx + scale * width + scale];
            } else {
                patch[idx] = 0.0f;
            }
        }

        __kernel void warpAffine(
            __global const uchar* img_ref,
            __global float* patch,
            float A00, float A01,
            float A10, float A11,
            float px_ref_x, float px_ref_y,
            int level_ref,
            int search_level,
            int pyramid_level,
            int halfpatch_size,
            int patch_size,
            int patch_size_total,
            int img_width,
            int img_height
        ) {
            int idx = get_global_id(0);
            if (idx >= patch_size_total) return;

            int x = idx % patch_size;
            int y = idx / patch_size;

            float px_patch_x = x - halfpatch_size;
            float px_patch_y = y - halfpatch_size;
            px_patch_x *= 1 << search_level;
            px_patch_y *= 1 << search_level;
            px_patch_x *= 1 << pyramid_level;
            px_patch_y *= 1 << pyramid_level;

            float px_x = A00 * px_patch_x + A01 * px_patch_y + px_ref_x;
            float px_y = A10 * px_patch_x + A11 * px_patch_y + px_ref_y;

            if (px_x >= 0 && px_x < img_width - 1 && px_y >= 0 && px_y < img_height - 1) {
                int px_x_i = (int)px_x;
                int px_y_i = (int)px_y;
                float subpix_x = px_x - px_x_i;
                float subpix_y = px_y - px_y_i;

                float w_tl = (1.0f - subpix_x) * (1.0f - subpix_y);
                float w_tr = subpix_x * (1.0f - subpix_y);
                float w_bl = (1.0f - subpix_x) * subpix_y;
                float w_br = subpix_x * subpix_y;

                int idx_tl = px_y_i * img_width + px_x_i;
                int idx_tr = px_y_i * img_width + px_x_i + 1;
                int idx_bl = (px_y_i + 1) * img_width + px_x_i;
                int idx_br = (px_y_i + 1) * img_width + px_x_i + 1;

                patch[idx] = w_tl * img_ref[idx_tl] + 
                            w_tr * img_ref[idx_tr] + 
                            w_bl * img_ref[idx_bl] + 
                            w_br * img_ref[idx_br];
            } else {
                patch[idx] = 0.0f;
            }
        }

        __kernel void calculateNCC(
            __global const float* ref_patch,
            __global const float* cur_patch,
            __global float* result,
            int patch_size
        ) {
            __local float sum_ref;
            __local float sum_cur;
            __local float numerator;
            __local float demoniator1;
            __local float demoniator2;

            if (get_local_id(0) == 0) {
                sum_ref = 0.0f;
                sum_cur = 0.0f;
                numerator = 0.0f;
                demoniator1 = 0.0f;
                demoniator2 = 0.0f;
            }

            barrier(CLK_LOCAL_MEM_FENCE);

            int idx = get_global_id(0);
            if (idx < patch_size) {
                float ref_val = ref_patch[idx];
                float cur_val = cur_patch[idx];
                atomic_add(&sum_ref, ref_val);
                atomic_add(&sum_cur, cur_val);
            }

            barrier(CLK_LOCAL_MEM_FENCE);

            float mean_ref = sum_ref / patch_size;
            float mean_cur = sum_cur / patch_size;

            if (idx < patch_size) {
                float ref_val = ref_patch[idx];
                float cur_val = cur_patch[idx];
                float ref_diff = ref_val - mean_ref;
                float cur_diff = cur_val - mean_cur;
                atomic_add(&numerator, ref_diff * cur_diff);
                atomic_add(&demoniator1, ref_diff * ref_diff);
                atomic_add(&demoniator2, cur_diff * cur_diff);
            }

            barrier(CLK_LOCAL_MEM_FENCE);

            if (get_local_id(0) == 0) {
                float denom = sqrt(demoniator1 * demoniator2 + 1e-10f);
                result[0] = numerator / denom;
            }
        }
    )";

    // 创建程序
    program_ = clCreateProgramWithSource(context_, 1, &kernel_code, NULL, NULL);
    if (!program_) {
        std::cerr << "Error: Failed to create program" << std::endl;
        return false;
    }

    // 构建程序
    cl_int err = clBuildProgram(program_, 1, &device_, NULL, NULL, NULL);
    if (err != CL_SUCCESS) {
        char buffer[4096];
        clGetProgramBuildInfo(program_, device_, CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, NULL);
        std::cerr << "Error: Failed to build program: " << buffer << std::endl;
        return false;
    }

    return true;
}

bool GPUAcceleration::createKernels() {
    // 创建内核
    kernel_getImagePatch_ = clCreateKernel(program_, "getImagePatch", NULL);
    kernel_warpAffine_ = clCreateKernel(program_, "warpAffine", NULL);
    kernel_calculateNCC_ = clCreateKernel(program_, "calculateNCC", NULL);

    if (!kernel_getImagePatch_ || !kernel_warpAffine_ || !kernel_calculateNCC_) {
        std::cerr << "Error: Failed to create kernels" << std::endl;
        return false;
    }

    return true;
}

void GPUAcceleration::getImagePatchGPU(const cv::Mat& img, const Vector2d& pc, float* patch_tmp, int level, int patch_size, int patch_size_total) {
    if (!supported_) {
        return;
    }

    // 创建图像缓冲区
    img_buffer_ = clCreateBuffer(context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, 
                               img.cols * img.rows * sizeof(uchar), (void*)img.data, NULL);

    // 创建补丁缓冲区
    patch_buffer_ = clCreateBuffer(context_, CL_MEM_WRITE_ONLY | CL_MEM_COPY_HOST_PTR, 
                                 patch_size_total * sizeof(float), patch_tmp, NULL);

    // 设置内核参数
    clSetKernelArg(kernel_getImagePatch_, 0, sizeof(cl_mem), &img_buffer_);
    clSetKernelArg(kernel_getImagePatch_, 1, sizeof(cl_mem), &patch_buffer_);
    clSetKernelArg(kernel_getImagePatch_, 2, sizeof(float), &pc[0]);
    clSetKernelArg(kernel_getImagePatch_, 3, sizeof(float), &pc[1]);
    clSetKernelArg(kernel_getImagePatch_, 4, sizeof(int), &level);
    clSetKernelArg(kernel_getImagePatch_, 5, sizeof(int), &patch_size);
    clSetKernelArg(kernel_getImagePatch_, 6, sizeof(int), &patch_size_total);
    clSetKernelArg(kernel_getImagePatch_, 7, sizeof(int), &img.cols);
    clSetKernelArg(kernel_getImagePatch_, 8, sizeof(int), &img.rows);

    // 执行内核
    size_t global_work_size = patch_size_total;
    clEnqueueNDRangeKernel(queue_, kernel_getImagePatch_, 1, NULL, &global_work_size, NULL, 0, NULL, NULL);

    // 读取结果
    clEnqueueReadBuffer(queue_, patch_buffer_, CL_TRUE, 0, patch_size_total * sizeof(float), patch_tmp, 0, NULL, NULL);

    // 释放缓冲区
    clReleaseMemObject(img_buffer_);
    clReleaseMemObject(patch_buffer_);
}

void GPUAcceleration::warpAffineGPU(const Matrix2d& A_cur_ref, const cv::Mat& img_ref, const Vector2d& px_ref, 
                                   int level_ref, int search_level, int pyramid_level, 
                                   int halfpatch_size, float* patch, int patch_size, int patch_size_total) {
    if (!supported_) {
        return;
    }

    // 创建图像缓冲区
    img_buffer_ = clCreateBuffer(context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, 
                               img_ref.cols * img_ref.rows * sizeof(uchar), (void*)img_ref.data, NULL);

    // 创建补丁缓冲区
    patch_buffer_ = clCreateBuffer(context_, CL_MEM_WRITE_ONLY | CL_MEM_COPY_HOST_PTR, 
                                 patch_size_total * sizeof(float), patch, NULL);

    // 计算 A_ref_cur (A_cur_ref 的逆)
    Matrix2d A_ref_cur = A_cur_ref.inverse();

    // 设置内核参数
    clSetKernelArg(kernel_warpAffine_, 0, sizeof(cl_mem), &img_buffer_);
    clSetKernelArg(kernel_warpAffine_, 1, sizeof(cl_mem), &patch_buffer_);
    clSetKernelArg(kernel_warpAffine_, 2, sizeof(float), &A_ref_cur(0, 0));
    clSetKernelArg(kernel_warpAffine_, 3, sizeof(float), &A_ref_cur(0, 1));
    clSetKernelArg(kernel_warpAffine_, 4, sizeof(float), &A_ref_cur(1, 0));
    clSetKernelArg(kernel_warpAffine_, 5, sizeof(float), &A_ref_cur(1, 1));
    clSetKernelArg(kernel_warpAffine_, 6, sizeof(float), &px_ref[0]);
    clSetKernelArg(kernel_warpAffine_, 7, sizeof(float), &px_ref[1]);
    clSetKernelArg(kernel_warpAffine_, 8, sizeof(int), &level_ref);
    clSetKernelArg(kernel_warpAffine_, 9, sizeof(int), &search_level);
    clSetKernelArg(kernel_warpAffine_, 10, sizeof(int), &pyramid_level);
    clSetKernelArg(kernel_warpAffine_, 11, sizeof(int), &halfpatch_size);
    clSetKernelArg(kernel_warpAffine_, 12, sizeof(int), &patch_size);
    clSetKernelArg(kernel_warpAffine_, 13, sizeof(int), &patch_size_total);
    clSetKernelArg(kernel_warpAffine_, 14, sizeof(int), &img_ref.cols);
    clSetKernelArg(kernel_warpAffine_, 15, sizeof(int), &img_ref.rows);

    // 执行内核
    size_t global_work_size = patch_size_total;
    clEnqueueNDRangeKernel(queue_, kernel_warpAffine_, 1, NULL, &global_work_size, NULL, 0, NULL, NULL);

    // 读取结果
    clEnqueueReadBuffer(queue_, patch_buffer_, CL_TRUE, 0, patch_size_total * sizeof(float), patch, 0, NULL, NULL);

    // 释放缓冲区
    clReleaseMemObject(img_buffer_);
    clReleaseMemObject(patch_buffer_);
}

double GPUAcceleration::calculateNCCGPU(float* ref_patch, float* cur_patch, int patch_size) {
    if (!supported_) {
        return 0.0;
    }

    // 创建缓冲区
    ref_patch_buffer_ = clCreateBuffer(context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, 
                                     patch_size * sizeof(float), ref_patch, NULL);
    cur_patch_buffer_ = clCreateBuffer(context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, 
                                     patch_size * sizeof(float), cur_patch, NULL);
    ncc_result_buffer_ = clCreateBuffer(context_, CL_MEM_WRITE_ONLY, sizeof(float), NULL, NULL);

    // 设置内核参数
    clSetKernelArg(kernel_calculateNCC_, 0, sizeof(cl_mem), &ref_patch_buffer_);
    clSetKernelArg(kernel_calculateNCC_, 1, sizeof(cl_mem), &cur_patch_buffer_);
    clSetKernelArg(kernel_calculateNCC_, 2, sizeof(cl_mem), &ncc_result_buffer_);
    clSetKernelArg(kernel_calculateNCC_, 3, sizeof(int), &patch_size);

    // 执行内核
    size_t global_work_size = patch_size;
    size_t local_work_size = 256;
    if (global_work_size < local_work_size) {
        local_work_size = global_work_size;
    }
    clEnqueueNDRangeKernel(queue_, kernel_calculateNCC_, 1, NULL, &global_work_size, &local_work_size, 0, NULL, NULL);

    // 读取结果
    float result;
    clEnqueueReadBuffer(queue_, ncc_result_buffer_, CL_TRUE, 0, sizeof(float), &result, 0, NULL, NULL);

    // 释放缓冲区
    clReleaseMemObject(ref_patch_buffer_);
    clReleaseMemObject(cur_patch_buffer_);
    clReleaseMemObject(ncc_result_buffer_);

    return result;
}

void GPUAcceleration::cleanup() {
    if (kernel_getImagePatch_) clReleaseKernel(kernel_getImagePatch_);
    if (kernel_warpAffine_) clReleaseKernel(kernel_warpAffine_);
    if (kernel_calculateNCC_) clReleaseKernel(kernel_calculateNCC_);
    if (program_) clReleaseProgram(program_);
    if (queue_) clReleaseCommandQueue(queue_);
    if (context_) clReleaseContext(context_);
}
