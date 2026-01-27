#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenGL and GLFW includes
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glu.h>

// Math includes
#include <Eigen/Eigen>

class LightweightVisualizer {
public:
    LightweightVisualizer() : window(nullptr) {
        // Initialize ROS
        nh.param<double>("point_size", point_size, 2.0);
        nh.param<double>("fov", fov, 60.0);
        nh.param<double>("near_plane", near_plane, 0.1);
        nh.param<double>("far_plane", far_plane, 1000.0);
        
        // Subscribe to optimized point cloud topic
        sub_pointcloud = nh.subscribe("/fast_livo/laser_cloud_optimized", 1, &LightweightVisualizer::pointcloudCallback, this);
        
        // Initialize GLFW
        if (!glfwInit()) {
            ROS_ERROR("Failed to initialize GLFW");
            return;
        }
        
        // Create window
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
        
        window = glfwCreateWindow(800, 600, "Lightweight PointCloud Visualizer", nullptr, nullptr);
        if (!window) {
            ROS_ERROR("Failed to create GLFW window");
            glfwTerminate();
            return;
        }
        
        glfwMakeContextCurrent(window);
        glfwSetWindowUserPointer(window, this);
        
        // Set callbacks
        glfwSetKeyCallback(window, keyCallback);
        glfwSetMouseButtonCallback(window, mouseButtonCallback);
        glfwSetCursorPosCallback(window, cursorPosCallback);
        
        // Initialize camera
        camera_position << 0.0, 0.0, 10.0;
        camera_front << 0.0, 0.0, -1.0;
        camera_up << 0.0, 1.0, 0.0;
        yaw = -90.0;
        pitch = 0.0;
        mouse_sensitivity = 0.1;
        zoom = 45.0;
        
        // Enable depth testing
        glEnable(GL_DEPTH_TEST);
        
        ROS_INFO("Lightweight Visualizer initialized successfully");
    }
    
    ~LightweightVisualizer() {
        if (window) {
            glfwDestroyWindow(window);
        }
        glfwTerminate();
    }
    
    void run() {
        if (!window) {
            return;
        }
        
        while (ros::ok() && !glfwWindowShouldClose(window)) {
            // Process ROS callbacks
            ros::spinOnce();
            
            // Render
            render();
            
            // Swap buffers
            glfwSwapBuffers(window);
            glfwPollEvents();
            
            // Limit frame rate
            usleep(16666); // ~60Hz
        }
    }
    
private:
    // ROS members
    ros::NodeHandle nh;
    ros::Subscriber sub_pointcloud;
    
    // Point cloud data
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    std::vector<float> point_data;
    std::vector<float> color_data;
    
    // GLFW window
    GLFWwindow* window;
    
    // Camera parameters
    Eigen::Vector3d camera_position;
    Eigen::Vector3d camera_front;
    Eigen::Vector3d camera_up;
    double yaw;
    double pitch;
    double mouse_sensitivity;
    double zoom;
    
    // Visualization parameters
    double point_size;
    double fov;
    double near_plane;
    double far_plane;
    
    // Mouse state
    bool first_mouse;
    double last_x;
    double last_y;
    
    // Point cloud callback
    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // Convert to PCL point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *new_cloud);
        
        // Update point data
        point_data.clear();
        color_data.clear();
        
        for (const auto& point : new_cloud->points) {
            // Add point coordinates
            point_data.push_back(point.x);
            point_data.push_back(point.y);
            point_data.push_back(point.z);
            
            // Add color based on intensity
            float intensity = point.intensity / 255.0;
            color_data.push_back(intensity);
            color_data.push_back(intensity * 0.7);
            color_data.push_back(intensity * 0.3);
        }
        
        cloud = new_cloud;
        ROS_INFO_THROTTLE(1.0, "Received point cloud with %zu points", cloud->size());
    }
    
    // Render function
    void render() {
        // Clear screen
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // Set up projection matrix
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        double aspect = static_cast<double>(width) / height;
        double fov_rad = fov * M_PI / 180.0;
        double top = near_plane * tan(fov_rad / 2.0);
        double bottom = -top;
        double right = top * aspect;
        double left = -right;
        glFrustum(left, right, bottom, top, near_plane, far_plane);
        
        // Set up model-view matrix
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        
        // Camera view
        Eigen::Vector3d center = camera_position + camera_front;
        gluLookAt(
            camera_position.x(), camera_position.y(), camera_position.z(),
            center.x(), center.y(), center.z(),
            camera_up.x(), camera_up.y(), camera_up.z()
        );
        
        // Draw coordinate axes
        drawAxes();
        
        // Draw point cloud
        if (!point_data.empty()) {
            glPointSize(point_size);
            glEnableClientState(GL_VERTEX_ARRAY);
            glEnableClientState(GL_COLOR_ARRAY);
            glVertexPointer(3, GL_FLOAT, 0, point_data.data());
            glColorPointer(3, GL_FLOAT, 0, color_data.data());
            glDrawArrays(GL_POINTS, 0, point_data.size() / 3);
            glDisableClientState(GL_VERTEX_ARRAY);
            glDisableClientState(GL_COLOR_ARRAY);
        }
    }
    
    // Draw coordinate axes
    void drawAxes() {
        float axis_length = 2.0f;
        
        // X-axis (red)
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(axis_length, 0.0f, 0.0f);
        glEnd();
        
        // Y-axis (green)
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, axis_length, 0.0f);
        glEnd();
        
        // Z-axis (blue)
        glColor3f(0.0f, 0.0f, 1.0f);
        glBegin(GL_LINES);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, axis_length);
        glEnd();
    }
    
    // Key callback
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
        LightweightVisualizer* viz = static_cast<LightweightVisualizer*>(glfwGetWindowUserPointer(window));
        if (!viz) return;
        
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, true);
        }
        
        // Camera movement
        double camera_speed = 0.1;
        if (key == GLFW_KEY_W && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
            viz->camera_position += camera_speed * viz->camera_front;
        }
        if (key == GLFW_KEY_S && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
            viz->camera_position -= camera_speed * viz->camera_front;
        }
        if (key == GLFW_KEY_A && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
            viz->camera_position -= (viz->camera_front.cross(viz->camera_up)).normalized() * camera_speed;
        }
        if (key == GLFW_KEY_D && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
            viz->camera_position += (viz->camera_front.cross(viz->camera_up)).normalized() * camera_speed;
        }
    }
    
    // Mouse button callback
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
        // Not implemented yet
    }
    
    // Cursor position callback
    static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
        LightweightVisualizer* viz = static_cast<LightweightVisualizer*>(glfwGetWindowUserPointer(window));
        if (!viz) return;
        
        if (viz->first_mouse) {
            viz->last_x = xpos;
            viz->last_y = ypos;
            viz->first_mouse = false;
        }
        
        double xoffset = xpos - viz->last_x;
        double yoffset = viz->last_y - ypos; // Reversed since y-coordinates go from bottom to top
        
        viz->last_x = xpos;
        viz->last_y = ypos;
        
        xoffset *= viz->mouse_sensitivity;
        yoffset *= viz->mouse_sensitivity;
        
        viz->yaw += xoffset;
        viz->pitch += yoffset;
        
        // Constrain pitch
        if (viz->pitch > 89.0) {
            viz->pitch = 89.0;
        }
        if (viz->pitch < -89.0) {
            viz->pitch = -89.0;
        }
        
        // Update front vector
        Eigen::Vector3d front;
        front.x() = cos(viz->yaw * M_PI / 180.0) * cos(viz->pitch * M_PI / 180.0);
        front.y() = sin(viz->pitch * M_PI / 180.0);
        front.z() = sin(viz->yaw * M_PI / 180.0) * cos(viz->pitch * M_PI / 180.0);
        viz->camera_front = front.normalized();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lightweight_visualizer");
    
    // Check if GLFW is available
    if (!glfwInit()) {
        ROS_ERROR("GLFW initialization failed. Please install GLFW3.");
        return 1;
    }
    glfwTerminate();
    
    LightweightVisualizer viz;
    viz.run();
    
    return 0;
}
