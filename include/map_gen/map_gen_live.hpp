#ifndef MAP_GEN_INCLUDE_MAP_GEN_LIVE_H_
#define MAP_GEN_INCLUDE_MAP_GEN_LIVE_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/ndt.h>

namespace map_gen {

class MapGenLive {
    public:
        explicit MapGenLive(rclcpp::Node::SharedPtr node, const std::string &ns = "",
                            std::vector<rclcpp::Parameter> parameters = {});
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    private:
        rclcpp::Node::SharedPtr node_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        std::unordered_map<std::string, rclcpp::Parameter> param_map_;
        void pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

}

#endif //MAP_GEN_INCLUDE_MAP_GEN_LIVE_H_