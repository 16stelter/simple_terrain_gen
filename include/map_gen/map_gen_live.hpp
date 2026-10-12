#ifndef MAP_GEN_INCLUDE_MAP_GEN_LIVE_H_
#define MAP_GEN_INCLUDE_MAP_GEN_LIVE_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
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

        bool is_paused_ = false;

        void pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

}

#endif //MAP_GEN_INCLUDE_MAP_GEN_LIVE_H_