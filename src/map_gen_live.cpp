#include "map_gen/map_gen_live.hpp"

namespace map_gen {

MapGenLive::MapGenLive(rclcpp::Node::SharedPtr node)
: node_(node),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_)
{
  node_->declare_parameter<std::string>("config_file", "");
  const std::string config_file = node_->get_parameter("config_file").as_string();
  try 
  {
    config_ = YAML::LoadFile(config_file);
  }
  catch (const std::exception &e) 
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load calibration file: %s", e.what());
  }

  map_.setGeometry(
    grid_map::Length(config_["grid_size_init"].as<double>(), config_["grid_size_init"].as<double>()),
    config_["grid_resolution"].as<double>(),
    grid_map::Position(0.0, 0.0)
  );
  map_.add("elevation");
  map_.add("color");
  map_.setFrameId(config_["grid_frame"].as<std::string>());

  pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    config_["pointcloud_topic"].as<std::string>(),
    rclcpp::SensorDataQoS(),
    std::bind(&MapGenLive::pointcloud_callback, this, std::placeholders::_1)
  );
  map_pub_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(
    config_["map_topic"].as<std::string>(), rclcpp::QoS(1).transient_local()
  );
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MapGenLive::publish_map, this));
}

void MapGenLive::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
{
  sensor_msgs::msg::PointCloud2 transformed_msg;
  geometry_msgs::msg::TransformStamped transform_stamped;
  try 
  {
    transform_stamped = tf_buffer_.lookupTransform(
      map_.getFrameId(), msg->header.frame_id, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
    tf2::doTransform(*msg, transformed_msg, transform_stamped);
  }
  catch (const tf2::TransformException &ex) 
  {
    RCLCPP_WARN(node_->get_logger(), "Could not transform point cloud: %s", ex.what());
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(transformed_msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iter_color(transformed_msg, "rgb");
  double alpha = config_["grid_alpha"].as<double>();
  int min_points = config_["grid_min_points"].as<int>();
  float camera_z = transform_stamped.transform.translation.z;

  struct Cell
  {
    int count = 0;
    float z = 0.0f;
    Eigen::Vector3f color = Eigen::Vector3f::Zero();
  };

  std::unordered_map<size_t, Cell> sum;
  auto size = map_.getSize();

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_color) 
  {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z) || *iter_z > camera_z) 
    {
      continue;
    }
    grid_map::Position position(*iter_x, *iter_y);
    grid_map::Index index;
    if (map_.getIndex(position, index)) 
    {
      uint32_t bits;
      std::memcpy(&bits, &(*iter_color), sizeof(bits));
      Cell &cell = sum[grid_map::getLinearIndexFromIndex(index, size)];
      cell.count++;
      cell.z += *iter_z;
      cell.color += Eigen::Vector3f(((bits >> 16) & 0xFF), ((bits >> 8) & 0xFF), (bits & 0xFF)) / 255.0f;
    }
  }
  for (const auto &[linearIndex, cell] : sum) 
  {
    if (cell.count >= min_points) 
    {
      grid_map::Index index = grid_map::getIndexFromLinearIndex(linearIndex, size);
      float &elevation = map_.at("elevation", index);
      float new_elevation = cell.z / cell.count;
      elevation = std::isnan(elevation) ? new_elevation : alpha * new_elevation + (1.0 - alpha) * elevation;
      
      float &color = map_.at("color", index);
      Eigen::Vector3f new_color = cell.color / cell.count;
      if (!std::isnan(color))
      {
        Eigen::Vector3f old_color;
        grid_map::colorValueToVector(color, old_color);
        new_color = alpha * new_color + (1.0 - alpha) * old_color;
      }
      grid_map::colorVectorToValue(new_color, color);
    }
  }
}

void MapGenLive::publish_map()
{
  std::unique_ptr<grid_map_msgs::msg::GridMap> map_msg = grid_map::GridMapRosConverter::toMessage(map_);
  map_pub_->publish(std::move(map_msg));
}

} // namespace map_gen

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("map_gen_live");
  [[maybe_unused]] map_gen::MapGenLive map_gen_live(node);
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
}