#ifndef MAP_GEN_INCLUDE_MAP_GEN_DEPTH_H_
#define MAP_GEN_INCLUDE_MAP_GEN_DEPTH_H_

#include <rclcpp/rclcpp.hpp>
#include "map_gen/srv/generate_map.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/IO/STL.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/Qt/Basic_viewer.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 Point;
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
            
            /*
            * Main function. Generates a map from the last received depth image.
            */
            void generate_map(const std::shared_ptr<map_gen::srv::GenerateMap::Request> request, std::shared_ptr<map_gen::srv::GenerateMap::Response> response);
            void depth_image_cb(const sensor_msgs::msg::Image::SharedPtr msg);
            void camera_info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    };
} //map_gen

#endif //MAP_GEN_INCLUDE_MAP_GEN_DEPTH_H_