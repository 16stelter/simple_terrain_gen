#ifndef MAP_GEN_INCLUDE_MAP_GEN_LIVE_H_
#define MAP_GEN_INCLUDE_MAP_GEN_LIVE_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QPushButton>
#include <QtCore/QTimer>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkPolyDataMapper.h>
#include <vtkNew.h>
#include <vtkSphereSource.h>
#include <vtkSTLWriter.h>

namespace map_gen {

class MapGenLive : public QMainWindow {
    public:
        explicit MapGenLive(rclcpp::Node::SharedPtr node, const std::string &ns = "",
                            std::vector<rclcpp::Parameter> parameters = {}, QWidget *parent = nullptr);
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_sub_;

    private:
        rclcpp::Node::SharedPtr node_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        std::unordered_map<std::string, rclcpp::Parameter> param_map_;
        vtkSmartPointer<vtkPoints> points_;
        QVTKOpenGLNativeWidget *vtk_widget_;

        double sx_;
        double sy_;
        double res_;
        int nx_;
        int ny_;

        std::array<double, 9> camera_intrinsics_;

        bool is_paused_ = false;

        void pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        void depth_image_cb(const sensor_msgs::msg::Image::SharedPtr msg);
        
        void depth_camera_info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        void update_surface_mesh(const std::vector<double> &z_sums, const std::vector<int> &z_counts);
};

}

#endif //MAP_GEN_INCLUDE_MAP_GEN_LIVE_H_