#ifndef MAP_GEN_INCLUDE_MAP_GEN_DEPTH_H_
#define MAP_GEN_INCLUDE_MAP_GEN_DEPTH_H_

#include <rclcpp/rclcpp.hpp>
#include "map_gen/srv/generate_map.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/IO/STL.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/Qt/Basic_viewer.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 Point;
typedef K::Aff_transformation_3 Transformation;
typedef CGAL::Surface_mesh<Point> Mesh;

namespace map_gen {
    class MapGenDepth {
        public:
            explicit MapGenDepth(rclcpp::Node::SharedPtr node, const std::string &ns = "",
                                          std::vector<rclcpp::Parameter> parameters = {});
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Service<map_gen::srv::GenerateMap>::SharedPtr map_gen_service_;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_cam_sub_;
            rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
            std::string camera_ns_;
            sensor_msgs::msg::Image depth_image_;
            std::array<double, 9> camera_intrinsics_;
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;
            
            /*
            * Map generation service. Generates a map from the last received depth image.
            */
            void generate_map(const std::shared_ptr<map_gen::srv::GenerateMap::Request> request, std::shared_ptr<map_gen::srv::GenerateMap::Response> response);

            /*
            * Callback for the depth image topic. Stores the last received depth image.
            */
            void depth_image_cb(const sensor_msgs::msg::Image::SharedPtr msg);

            /*
            * Callback for the camera info topic. Stores the last received camera intrinsics.
            */
            void camera_info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    };
} //map_gen

#endif //MAP_GEN_INCLUDE_MAP_GEN_DEPTH_H_