#ifndef MAP_GEN_INCLUDE_MAP_GEN_LIVE_H_
#define MAP_GEN_INCLUDE_MAP_GEN_LIVE_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <yaml-cpp/yaml.h>

namespace map_gen {
class MapGenLive {
  public:
    explicit MapGenLive(rclcpp::Node::SharedPtr node);
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

  private:
    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    YAML::Node config_;

    grid_map::GridMap map_;

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publish_map();
};
}

#endif //MAP_GEN_INCLUDE_MAP_GEN_LIVE_H_