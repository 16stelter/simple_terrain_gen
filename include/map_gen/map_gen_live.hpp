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
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpinBox>
#include <QtCore/QTimer>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkPolyDataMapper.h>
#include <vtkNew.h>
#include <vtkCommand.h>
#include <vtkPointPicker.h>
#include <vtkCallbackCommand.h>
#include <vtkSTLWriter.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkIntArray.h>
#include <vtkInteractorStyle.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkLookupTable.h>
#include <vtkPointData.h>



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
        int draw_mode_ = 0; // 0: off, 1: draw, 2: erase
        int draw_type_ = 0;
        int brush_size_ = 1;
        bool is_mousedown_ = false;

        vtkSmartPointer<vtkIntArray> terrain_map_;

        void pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        void depth_image_cb(const sensor_msgs::msg::Image::SharedPtr msg);
        
        void depth_camera_info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        void update_surface_mesh(const std::vector<double> &z_sums, const std::vector<int> &z_counts);
};

class ColorMap
{
  public:
    using Color = std::tuple<float, float, float>;
    static std::unordered_map<int, Color>& Get()
    {
      static std::unordered_map<int, Color> color_map = 
      {
        {0, {0.6784314f, 0.8470588f, 0.9019608f}}, // Light Blue
        {1,  {1.0f, 1.0f, 0.0f}},                 // Yellow
        {2, {0.5019608f, 0.0f, 0.5019608f}},     // Purple
        {3,  {0.5019608f, 0.0f, 0.0f}},           // Dark Red
        {4,  {1.0f, 0.0f, 0.0f}},                 // Red
        {5,  {0.0f, 1.0f, 0.0f}},                 // Green
        {6,  {0.0f, 0.0f, 1.0f}},                 // Blue
        {7, {1.0f, 0.6470588f, 0.0f}},           // Orange
        {8,  {0.0f, 1.0f, 1.0f}},                 // Cyan
        {9,  {1.0f, 0.0f, 1.0f}},                 // Magenta
        {10,  {0.0f, 0.5019608f, 0.0f}},           // Dark Green
        {11, {0.0f, 0.0f, 0.5019608f}},            // Dark Blue
        {12, {0.0f, 0.5019608f, 0.5019608f}},     // Teal
        {13, {0.8235294f, 0.4117647f, 0.1176471f}}, // Chocolate
        {14,  {1.0f, 1.0f, 1.0f}},                 // White
        {15, {0.2941177f, 0.0f, 0.5098039f}}      // Indigo
      };
      return color_map;
    }
};

}

#endif //MAP_GEN_INCLUDE_MAP_GEN_LIVE_H_