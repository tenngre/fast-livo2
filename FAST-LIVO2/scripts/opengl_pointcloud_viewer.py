#!/usr/bin/env python3

import rospy
import numpy as np
import struct
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

class OpenGLPointCloudViewer:
    def __init__(self):
        # 点云数据存储
        self.points = []
        self.colors = []
        self.point_size = 2.0
        
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
        # glEnable(GL_POINT_SMOOTH)
        # glHint(GL_POINT_SMOOTH_HINT, GL_NICEST)
        # glEnable(GL_BLEND)
        # glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    
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
                    # 使用方法1：将float转换为int，然后提取RGB
                    # 这是从分析结果中确定的正确方法
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
        
        # 更新点云数据
        self.points = np.array(points, dtype=np.float32)
        self.colors = np.array(colors, dtype=np.float32)
    
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
        
        # 绘制点云
        if len(self.points) > 0:
            self.draw_point_cloud()
        
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
    
    def draw_point_cloud(self):
        # 设置点大小
        glPointSize(self.point_size)
        
        # 使用顶点数组绘制点云
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_COLOR_ARRAY)
        
        glVertexPointer(3, GL_FLOAT, 0, self.points)
        glColorPointer(3, GL_FLOAT, 0, self.colors)
        
        glDrawArrays(GL_POINTS, 0, len(self.points))
        
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
    
    def idle(self):
        # 空闲回调，用于更新显示
        glutPostRedisplay()
    
    def run(self):
        # 运行主循环
        glutMainLoop()

if __name__ == '__main__':
    rospy.init_node('opengl_pointcloud_viewer')
    viewer = OpenGLPointCloudViewer()
    viewer.run()
