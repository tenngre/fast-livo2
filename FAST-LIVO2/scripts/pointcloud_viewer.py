#!/usr/bin/env python3

import rospy
import numpy as np
import struct
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

class PointCloudViewer:
    def __init__(self):
        # 点云数据存储
        self.current_points = np.array([], dtype=np.float32)
        self.current_colors = np.array([], dtype=np.float32)
        self.accumulated_points = np.array([], dtype=np.float32)
        self.accumulated_colors = np.array([], dtype=np.float32)
        self.point_size = 2.0
        
        # 点云累积参数
        self.accumulation_enabled = True
        self.max_points = 100000  # 最大累积点数
        self.voxel_size = 0.05  # 体素滤波大小，用于去重
        self.accumulation_rate = 0.5  # 累积速率，控制新点云的加入比例
        
        # 相机参数
        self.camera_pos = [0.0, -5.0, 2.0]
        self.camera_target = [0.0, 0.0, 0.0]
        self.camera_up = [0.0, 0.0, 1.0]
        
        # 窗口参数
        self.window_width = 800
        self.window_height = 600
        self.rotation_x = 0.0
        self.rotation_y = 0.0
        self.last_mouse_x = 0
        self.last_mouse_y = 0
        self.mouse_down = False
        
        # ROS订阅
        rospy.Subscriber('/cloud_registered', PointCloud2, self.point_cloud_callback)
        
        # 初始化OpenGL
        glutInit()
        glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH)
        glutInitWindowSize(self.window_width, self.window_height)
        glutCreateWindow('FAST-LIVO2 PointCloud Viewer')
        
        # 注册回调函数
        glutDisplayFunc(self.display)
        glutReshapeFunc(self.reshape)
        glutMouseFunc(self.mouse)
        glutMotionFunc(self.motion)
        glutKeyboardFunc(self.keyboard)
        glutIdleFunc(self.idle)
        
        # 初始化OpenGL设置
        self.init_opengl()
    
    def init_opengl(self):
        glClearColor(0.1, 0.1, 0.1, 1.0)
        glEnable(GL_DEPTH_TEST)
        # 禁用点平滑和混合以提高软件渲染性能
    
    def point_cloud_callback(self, msg):
        # 解析点云数据
        points = []
        colors = []
        
        # 下采样因子，减少点云数量（软件渲染环境下使用更大的因子）
        downsample_factor = 4
        count = 0
        
        # 记录处理时间
        start_time = rospy.Time.now()
        
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True):
            if count % downsample_factor == 0:
                # 添加点坐标
                points.append([p[0], p[1], p[2]])
                
                # 处理颜色信息
                if len(p) > 3 and p[3] is not None:
                    # 从float类型的rgb值中提取颜色
                    rgb = p[3]
                    try:
                        # 将float32转换为uint32
                        rgb_int = struct.unpack('I', struct.pack('f', rgb))[0]
                        # 提取RGB分量
                        r = ((rgb_int >> 16) & 0xff) / 255.0
                        g = ((rgb_int >> 8) & 0xff) / 255.0
                        b = (rgb_int & 0xff) / 255.0
                        # 添加颜色
                        colors.append([r, g, b])
                    except Exception as e:
                        # 如果出现错误，使用默认颜色
                        colors.append([0.5, 0.5, 1.0])
                else:
                    # 默认颜色
                    colors.append([0.5, 0.5, 1.0])
            count += 1
        
        # 计算处理时间
        process_time = (rospy.Time.now() - start_time).to_sec()
        # 每10帧打印一次性能信息
        if count % 10 == 0:
            rospy.loginfo("Point cloud processing: %d points in %.3f seconds (%.1f FPS)", 
                         len(points), process_time, 1.0/process_time if process_time > 0 else 0)
        
        # 更新当前点云数据
        self.current_points = np.array(points, dtype=np.float32)
        self.current_colors = np.array(colors, dtype=np.float32)
        
        # 如果启用了累积功能，将新点云添加到累积点云中
        if self.accumulation_enabled:
            self.accumulate_point_cloud()
    
    def accumulate_point_cloud(self):
        """累积点云数据，添加新点云到历史点云中"""
        if len(self.current_points) == 0:
            return
        
        # 计算需要添加的点数
        num_points_to_add = int(len(self.current_points) * self.accumulation_rate)
        if num_points_to_add == 0:
            return
        
        # 随机选择一部分新点云添加到累积点云中
        indices = np.random.choice(len(self.current_points), num_points_to_add, replace=False)
        new_points = self.current_points[indices]
        new_colors = self.current_colors[indices]
        
        # 将新点云添加到累积点云中
        if len(self.accumulated_points) == 0:
            self.accumulated_points = new_points
            self.accumulated_colors = new_colors
        else:
            # 合并点云和颜色
            combined_points = np.vstack((self.accumulated_points, new_points))
            combined_colors = np.vstack((self.accumulated_colors, new_colors))
            
            # 应用体素滤波去重
            filtered_points, filtered_colors = self.voxel_filter(combined_points, combined_colors)
            
            # 限制最大点数
            if len(filtered_points) > self.max_points:
                # 随机采样到最大点数
                sample_indices = np.random.choice(len(filtered_points), self.max_points, replace=False)
                self.accumulated_points = filtered_points[sample_indices]
                self.accumulated_colors = filtered_colors[sample_indices]
            else:
                self.accumulated_points = filtered_points
                self.accumulated_colors = filtered_colors
        
        rospy.loginfo("Accumulated point cloud: %d points", len(self.accumulated_points))
    
    def voxel_filter(self, points, colors):
        """应用体素滤波去重点云"""
        if len(points) == 0:
            return points, colors
        
        # 简单的体素滤波实现
        # 创建体素字典，键是体素坐标，值是该体素内的点和颜色
        voxel_dict = {}
        
        for i, point in enumerate(points):
            # 计算体素坐标
            voxel_coord = tuple(np.floor(point / self.voxel_size).astype(int))
            
            # 如果体素不存在，添加新点
            if voxel_coord not in voxel_dict:
                voxel_dict[voxel_coord] = (point, colors[i])
        
        # 从体素字典中提取去重后的点云和颜色
        filtered_points = []
        filtered_colors = []
        for point, color in voxel_dict.values():
            filtered_points.append(point)
            filtered_colors.append(color)
        
        return np.array(filtered_points, dtype=np.float32), np.array(filtered_colors, dtype=np.float32)
    
    def display(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        # 设置相机
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(
            self.camera_pos[0], self.camera_pos[1], self.camera_pos[2],
            self.camera_target[0], self.camera_target[1], self.camera_target[2],
            self.camera_up[0], self.camera_up[1], self.camera_up[2]
        )
        
        # 应用旋转
        glRotatef(self.rotation_x, 1.0, 0.0, 0.0)
        glRotatef(self.rotation_y, 0.0, 1.0, 0.0)
        
        # 绘制坐标系
        self.draw_coordinate_system()
        
        # 绘制累积点云（历史点云）
        if len(self.accumulated_points) > 0:
            # 使用稍微小一点的点大小绘制累积点云
            original_point_size = self.point_size
            self.point_size = original_point_size * 0.8
            self.draw_point_cloud(self.accumulated_points, self.accumulated_colors)
            self.point_size = original_point_size
        
        # 绘制当前点云
        if len(self.current_points) > 0:
            self.draw_point_cloud(self.current_points, self.current_colors)
        
        glutSwapBuffers()
    
    def draw_coordinate_system(self):
        # 绘制坐标轴
        glLineWidth(2.0)
        
        # X轴 (红色)
        glColor3f(1.0, 0.0, 0.0)
        glBegin(GL_LINES)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(1.0, 0.0, 0.0)
        glEnd()
        
        # Y轴 (绿色)
        glColor3f(0.0, 1.0, 0.0)
        glBegin(GL_LINES)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 1.0, 0.0)
        glEnd()
        
        # Z轴 (蓝色)
        glColor3f(0.0, 0.0, 1.0)
        glBegin(GL_LINES)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 1.0)
        glEnd()
    
    def draw_point_cloud(self, points=None, colors=None):
        # 如果没有提供点云和颜色，使用当前点云
        if points is None:
            points = self.current_points
        if colors is None:
            colors = self.current_colors
        
        # 检查点云是否为空
        if len(points) == 0:
            return
        
        # 直接使用原始点云，不进行任何优化
        filtered_points, filtered_colors = points, colors
        
        # 检查过滤后的点云是否为空
        if len(filtered_points) == 0:
            return
        
        # 设置点大小
        glPointSize(self.point_size)
        
        # 使用顶点数组绘制点云
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_COLOR_ARRAY)
        
        glVertexPointer(3, GL_FLOAT, 0, filtered_points)
        glColorPointer(3, GL_FLOAT, 0, filtered_colors)
        
        glDrawArrays(GL_POINTS, 0, len(filtered_points))
        
        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_COLOR_ARRAY)
    
    def reshape(self, width, height):
        self.window_width = width
        self.window_height = height
        glViewport(0, 0, width, height)
        
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, float(width)/float(height), 0.1, 100.0)
    
    def mouse(self, button, state, x, y):
        if button == GLUT_LEFT_BUTTON:
            if state == GLUT_DOWN:
                self.mouse_down = True
                self.last_mouse_x = x
                self.last_mouse_y = y
            else:
                self.mouse_down = False
        elif button == 3:  # 滚轮上滚
            self.camera_pos[1] += 0.1
        elif button == 4:  # 滚轮下滚
            self.camera_pos[1] -= 0.1
    
    def motion(self, x, y):
        if self.mouse_down:
            dx = x - self.last_mouse_x
            dy = y - self.last_mouse_y
            
            self.rotation_y += dx * 0.5
            self.rotation_x += dy * 0.5
            
            self.last_mouse_x = x
            self.last_mouse_y = y
    
    def keyboard(self, key, x, y):
        # 键盘控制
        if key == b'w':
            self.camera_pos[1] += 0.1
        elif key == b's':
            self.camera_pos[1] -= 0.1
        elif key == b'a':
            self.camera_pos[0] -= 0.1
        elif key == b'd':
            self.camera_pos[0] += 0.1
        elif key == b'q':
            self.camera_pos[2] -= 0.1
        elif key == b'e':
            self.camera_pos[2] += 0.1
        elif key == b'=':
            self.point_size += 0.5
        elif key == b'-':
            if self.point_size > 0.5:
                self.point_size -= 0.5
        elif key == b'x':
            rospy.signal_shutdown('User exit')
            glutLeaveMainLoop()
        elif key == b'c':
            # 清除累积点云
            self.accumulated_points = []
            self.accumulated_colors = []
            rospy.loginfo("Cleared accumulated point cloud")
        elif key == b'k':
            # 切换累积功能
            self.accumulation_enabled = not self.accumulation_enabled
            status = "enabled" if self.accumulation_enabled else "disabled"
            rospy.loginfo("Point cloud accumulation %s", status)
        elif key == b'r':
            # 重置相机位置
            self.camera_pos = [0.0, -5.0, 2.0]
            self.camera_target = [0.0, 0.0, 0.0]
            self.rotation_x = 0.0
            self.rotation_y = 0.0
            rospy.loginfo("Reset camera position")
    
    def idle(self):
        # 空闲回调，用于更新显示
        glutPostRedisplay()
    
    def run(self):
        # 运行主循环
        glutMainLoop()

if __name__ == '__main__':
    rospy.init_node('pointcloud_viewer')
    viewer = PointCloudViewer()
    viewer.run()
