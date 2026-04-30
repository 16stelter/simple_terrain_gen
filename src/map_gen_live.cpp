#include "map_gen/map_gen_live.hpp"

namespace map_gen {

MapGenLive::MapGenLive(rclcpp::Node::SharedPtr node,
                       const std::string &ns,
                       std::vector<rclcpp::Parameter> parameters)
: node_(node),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_)
  {
    node_->declare_parameter<std::string>("pointcloud_topic", "");
    node_->declare_parameter<bool>("use_intensity", false);
    node_->declare_parameter<bool>("use_rgb", false);
    node_->declare_parameter<double>("grid_resolution", 1.0);
    node_->declare_parameter<double>("grid_size_x", 10.0);
    node_->declare_parameter<double>("grid_size_y", 10.0);
    std::vector<std::string> param_names = {"pointcloud_topic", 
                                            "use_intensity", 
                                            "use_rgb",
                                            "grid_resolution",
                                            "grid_size_x",
                                            "grid_size_y"};
    std::vector<rclcpp::Parameter> params_ = node_->get_parameters(param_names);
    for (const auto& param : params_) {
        param_map_[param.get_name()] = param;
    }

    pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        param_map_["pointcloud_topic"].as_string(), 10,
        std::bind(&MapGenLive::pointcloud_cb, this, std::placeholders::_1)
    );
  }

  void MapGenLive::pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(node_->get_logger(), "Getting data.");
  }
}

int main(int argc, char **argv) 
{
  // init node
  rclcpp::init(argc, argv);

  // Create ros node
  auto node = std::make_shared<rclcpp::Node>("map_gen_live");

  // Create cpp node
  [[maybe_unused]] map_gen::MapGenLive map_gen_live(node);

  // Create executor
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  // Spin executor to process callbacks
  exec.spin();
  rclcpp::shutdown();
}