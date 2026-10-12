#include "map_gen/map_gen_live.hpp"

namespace map_gen {

MapGenLive::MapGenLive(rclcpp::Node::SharedPtr node,
                       const std::string &ns,
                       std::vector<rclcpp::Parameter> parameters,
                       QWidget *parent)
: QMainWindow(parent),
  node_(node),
  tf_buffer_(node_->get_clock()),
  tf_listener_(tf_buffer_)
  {

  // Declare parameters
  node_->declare_parameter<double>("grid_resolution", 1.0);
  node_->declare_parameter<double>("grid_size_x", 10.0);
  node_->declare_parameter<double>("grid_size_y", 10.0);
  node_->declare_parameter<bool>("use_tf", true);
  node_->declare_parameter<std::string>("pointcloud_topic", "");
  node_->declare_parameter<std::string>("depth_image_topic", "");
  node_->declare_parameter<std::string>("depth_camera_info_topic", "");
  node_->declare_parameter<std::vector<std::string>>("terrain_types", std::vector<std::string>{"soil", "bedrock", "sand", "large_rock"});
  std::vector<std::string> param_names = {"grid_resolution",
                                          "grid_size_x",
                                          "grid_size_y",
                                          "use_tf",
                                          "pointcloud_topic",
                                          "depth_image_topic",
                                          "depth_camera_info_topic",
                                          "terrain_types"};
  std::vector<rclcpp::Parameter> params_ = node_->get_parameters(param_names);
  for (const auto& param : params_) {
      param_map_[param.get_name()] = param;
  }

  // Ros subscriptions
  if (!param_map_["pointcloud_topic"].as_string().empty()) {
    pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        param_map_["pointcloud_topic"].as_string(),
        10,
        std::bind(&MapGenLive::pointcloud_cb, this, std::placeholders::_1));
  }
  else if (!param_map_["depth_image_topic"].as_string().empty() && !param_map_["depth_camera_info_topic"].as_string().empty()) {
    depth_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        param_map_["depth_image_topic"].as_string(),
        10,
        std::bind(&MapGenLive::depth_image_cb, this, std::placeholders::_1));
    depth_camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        param_map_["depth_camera_info_topic"].as_string(),
        10,
        std::bind(&MapGenLive::depth_camera_info_cb, this, std::placeholders::_1));
  }
  else {
    RCLCPP_ERROR(node_->get_logger(), "No valid input topic specified. Please set either pointcloud_topic or depth_image_topic.");
  }

  // Qt and VTK setup
  resize(800, 600);
  setWindowTitle("MapGen Live");
  vtk_widget_ = new QVTKOpenGLNativeWidget(this);
  setCentralWidget(vtk_widget_);
  vtkNew<vtkGenericOpenGLRenderWindow> render_window;
  vtk_widget_->setRenderWindow(render_window);
  vtkNew<vtkRenderer> renderer;
  render_window->AddRenderer(renderer);

  // Toolbar setup
  QToolBar *toolbar = new QToolBar(this);
  addToolBar(toolbar);
  QPushButton *pause_button = new QPushButton("Pause", this);
  toolbar->addWidget(pause_button);
  connect(pause_button, &QPushButton::clicked, this, [this, pause_button]() {
      is_paused_ = !is_paused_;
      pause_button->setText(is_paused_ ? "Resume" : "Pause");
  });
  QPushButton *draw_button = new QPushButton("Draw", this);
  toolbar->addWidget(draw_button);
  QPushButton *erase_button = new QPushButton("Erase", this);
  toolbar->addWidget(erase_button);
  connect(draw_button, &QPushButton::clicked, this, [this, draw_button, erase_button]() {
    draw_mode_ == 1 ? draw_mode_ = 0 : draw_mode_ = 1;
    draw_button->setText(draw_mode_ == 1 ? "Stop Drawing" : "Draw");
    erase_button->setText("Erase");
  });
  connect(erase_button, &QPushButton::clicked, this, [this, erase_button, draw_button]() {
    draw_mode_ == 2 ? draw_mode_ = 0 : draw_mode_ = 2;
    erase_button->setText(draw_mode_ == 2 ? "Stop Erasing" : "Erase");
    draw_button->setText("Draw");
  });
  QComboBox *dropdown = new QComboBox(this);
  std::vector<std::string> terrain_types = param_map_["terrain_types"].as_string_array();
  for (std::string val : terrain_types) {
    dropdown->addItem(QString::fromStdString(val));
  }
  toolbar->addWidget(dropdown);
  connect(dropdown, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
    draw_type_ = index;
  });

  // painting
  vtkNew<vtkLookupTable> lut;
  int num_terrains = terrain_types.size();
  lut->SetNumberOfTableValues(num_terrains);
  
  auto& color_map = ColorMap::Get();
  
  for (int i = 0; i < num_terrains; ++i) {
    if (color_map.find(i) != color_map.end()) {
      auto color = color_map[i];
      lut->SetTableValue(i, std::get<0>(color), std::get<1>(color), std::get<2>(color));
    } else {
      lut->SetTableValue(i, 1.0, 1.0, 1.0);
    }
  }
  lut->Build();

  vtkNew<vtkCallbackCommand> click_callback;
  click_callback->SetClientData(this);
  click_callback->SetCallback([](vtkObject* caller, long unsigned int event_id, void* client_data, void*) {
    auto* self = static_cast<MapGenLive*>(client_data);
    auto* style = vtkInteractorStyle::SafeDownCast(caller);
    auto* interactor = style->GetInteractor();
    bool handled = false;
    if (self->draw_mode_ != 0) {
      if (event_id == vtkCommand::LeftButtonPressEvent) {
        self->is_mousedown_ = true;
        handled = true;
      }
      else if (event_id == vtkCommand::LeftButtonReleaseEvent) {
        self->is_mousedown_ = false;
        handled = true;
      }
      else if (event_id == vtkCommand::MouseMoveEvent && self->is_mousedown_) {
        handled = true;
      }

      if (handled && self->is_mousedown_) {
        int* pos = interactor->GetEventPosition();
        vtkNew<vtkPointPicker> picker;
        picker->Pick(pos[0], pos[1], 0, self->vtk_widget_->renderWindow()->GetRenderers()->GetFirstRenderer());
        vtkIdType point_id = picker->GetPointId();
        if (point_id != -1) {
          self->draw_mode_ == 1 ? self->terrain_map_->SetValue(point_id, self->draw_type_) : self->terrain_map_->SetValue(point_id, 0);
          self->terrain_map_->Modified();
          self->vtk_widget_->renderWindow()->Render();
        }
      }
    }
    if (!handled) {
      if (event_id == vtkCommand::LeftButtonPressEvent) {
        style->OnLeftButtonDown();
      } else if (event_id == vtkCommand::LeftButtonReleaseEvent) {
        style->OnLeftButtonUp();
      } else if (event_id == vtkCommand::MouseMoveEvent) {
        style->OnMouseMove();
      }
    }
  });
  auto* interactor_style = render_window->GetInteractor()->GetInteractorStyle();
  interactor_style->RemoveObservers(vtkCommand::LeftButtonPressEvent);
  interactor_style->RemoveObservers(vtkCommand::LeftButtonReleaseEvent);
  interactor_style->RemoveObservers(vtkCommand::MouseMoveEvent);
  interactor_style->AddObserver(vtkCommand::LeftButtonPressEvent, click_callback);
  interactor_style->AddObserver(vtkCommand::LeftButtonReleaseEvent, click_callback);
  interactor_style->AddObserver(vtkCommand::MouseMoveEvent, click_callback);

  // Mesh setup
  points_ = vtkSmartPointer<vtkPoints>::New();
  sx_ = param_map_["grid_size_x"].as_double();
  sy_ = param_map_["grid_size_y"].as_double();
  res_ = param_map_["grid_resolution"].as_double();
  nx_ = sx_ / res_;
  ny_ = sy_ / res_;

  for (int y = 0; y < ny_; ++y) {
    for (int x = 0; x < nx_; ++x) {
      points_->InsertNextPoint(x * res_,
                                y * res_,
                                0.0);
    }
  }
  vtkNew<vtkCellArray> polys;
  for (int y = 0; y < ny_ - 1; ++y) {
    for (int x = 0; x < nx_ - 1; ++x) {
      vtkIdType pt1 = y * nx_ + x;
      vtkIdType pt2 = pt1 + 1;
      vtkIdType pt3 = pt1 + nx_;
      vtkIdType pt4 = pt3 + 1;
      polys->InsertNextCell(3);
      polys->InsertCellPoint(pt1);
      polys->InsertCellPoint(pt2);
      polys->InsertCellPoint(pt3);
      polys->InsertNextCell(3);
      polys->InsertCellPoint(pt2);
      polys->InsertCellPoint(pt4);
      polys->InsertCellPoint(pt3);
    }
  }
  vtkNew<vtkPolyData> surface_mesh;
  surface_mesh->SetPoints(points_);
  surface_mesh->SetPolys(polys);
  terrain_map_ = vtkSmartPointer<vtkIntArray>::New(); // this holds the terrain/color data
  terrain_map_->SetNumberOfComponents(1);
  terrain_map_->SetName("TerrainType");
  for (int i = 0; i < nx_ * ny_; ++i) {
    terrain_map_->InsertNextValue(0);
  }
  surface_mesh->GetPointData()->SetScalars(terrain_map_);

  // Create mapper and actor
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(surface_mesh);
  mapper->SetScalarRange(0, std::max(1, num_terrains - 1));
  mapper->SetLookupTable(lut);
  mapper->SetScalarModeToUsePointData();
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  renderer->AddActor(actor);
  renderer->ResetCamera();
}

void MapGenLive::pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (is_paused_) {
    return;
  }
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  if (!param_map_["use_tf"].as_bool()) {
    RCLCPP_WARN_ONCE(node_->get_logger(), "TF is disabled. Point cloud will not be transformed.");
    transformed_cloud = *msg;
  }
  else {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
          "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_ONCE(node_->get_logger(), "Could not transform point cloud: %s", ex.what());
      return;
    }

    tf2::doTransform(*msg, transformed_cloud, transform_stamped);
  }

  std::vector<double> z_sums(nx_ * ny_, 0.0);
  std::vector<int> z_counts(nx_ * ny_, 0);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(transformed_cloud, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    int x_idx = static_cast<int>((*iter_x) / res_);
    int y_idx = static_cast<int>((*iter_y) / res_);
    if (x_idx >= 0 && x_idx < nx_ && y_idx >= 0 && y_idx < ny_) {
      int idx = y_idx * nx_ + x_idx;
      z_sums[idx] += *iter_z;
      ++z_counts[idx];
    }
  }

  update_surface_mesh(z_sums, z_counts);
}

void MapGenLive::depth_image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (is_paused_ || camera_intrinsics_.empty()) {
      return;
  }

  if (msg->encoding != "16UC1" && msg->encoding != "32FC1") {
    RCLCPP_ERROR(node_->get_logger(), "Unsupported depth image encoding: %s", msg->encoding.c_str());
    return;
  }

  tf2::Transform tf2_transform;
  tf2_transform.setIdentity();
  if (!param_map_["use_tf"].as_bool()) {
    RCLCPP_WARN_ONCE(node_->get_logger(), "TF is disabled. Depth image will not be transformed.");
  }
  else {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
          "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
      tf2::fromMsg(transform_stamped.transform, tf2_transform);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_ONCE(node_->get_logger(), "Could not transform depth image: %s", ex.what());
      return;
    }
  }

  std::vector<double> z_sums(nx_ * ny_, 0.0);
  std::vector<int> z_counts(nx_ * ny_, 0);

  for (size_t v = 0; v < msg->height; ++v) {
    for (size_t u = 0; u < msg->width; ++u) {
      float z = 0.0f;
      if (msg->encoding == "16UC1") {
        uint16_t depth_raw = *reinterpret_cast<const uint16_t*>(&msg->data[v * msg->step + u * 2]);
        z = static_cast<float>(depth_raw) / 1000.0; // Convert mm to meters
      } else if (msg->encoding == "32FC1") {
        z = *reinterpret_cast<const float*>(&msg->data[v * msg->step + u * 4]);
      }
      if (std::isnan(z) || z <= 0.0f) {
        continue;
      }

      tf2::Vector3 point_camera((u - camera_intrinsics_[2]) * z / camera_intrinsics_[0],
                                (v - camera_intrinsics_[5]) * z / camera_intrinsics_[4],
                                z);
      tf2::Vector3 point_map = tf2_transform * point_camera;

      int x_idx = static_cast<int>(point_map.x() / res_);
      int y_idx = static_cast<int>(point_map.y() / res_);

      if (x_idx >= 0 && x_idx < nx_ && y_idx >= 0 && y_idx < ny_) {
        int idx = y_idx * nx_ + x_idx;
        z_sums[idx] += point_map.z();
        ++z_counts[idx];
      }
    }
  }
  update_surface_mesh(z_sums, z_counts);
}

void MapGenLive::depth_camera_info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  camera_intrinsics_ = msg->k;
  RCLCPP_INFO_ONCE(node_->get_logger(), "Received camera intrinsics: fx=%f, fy=%f, cx=%f, cy=%f", camera_intrinsics_[0], camera_intrinsics_[4], camera_intrinsics_[2], camera_intrinsics_[5]);
}

void MapGenLive::update_surface_mesh(const std::vector<double> &z_sums, const std::vector<int> &z_counts)
{
  std::vector<double> z_averages(nx_ * ny_, 0.0);
  for (int i = 0; i < nx_ * ny_; ++i) {
    if (z_counts[i] > 0) {
      z_averages[i] = z_sums[i] / z_counts[i];
    }
  }

  for (int y = 0; y < ny_; ++y) {
    for (int x = 0; x < nx_; ++x) {
      int idx = y * nx_ + x;
      if (z_counts[idx] == 0) {
        double neighbor_sum = 0.0;
        int neighbor_count = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            int ny_idx = y + dy;
            int nx_idx = x + dx;
            if (nx_idx >= 0 && nx_idx < nx_ && ny_idx >= 0 && ny_idx < ny_) {
              int ni = ny_idx * nx_ + nx_idx;
              if (z_counts[ni] > 0) {
                neighbor_sum += z_sums[ni] / z_counts[ni];
                ++neighbor_count;
              }
            }
          }
        }
        if (neighbor_count > 0) {
          z_averages[idx] = neighbor_sum / neighbor_count;
        }
      }
    }
  }

  QMetaObject::invokeMethod(this, [this, z_averages]() {
    for (int y = 0; y < ny_; ++y) {
      for (int x = 0; x < nx_; ++x) {
        double point[3];
        int idx = y * nx_ + x;
        points_->GetPoint(idx, point);
        point[2] = z_averages[idx];
        points_->SetPoint(idx, point);
      }
    }
    points_->Modified();
    vtk_widget_->renderWindow()->Render();
  }, Qt::QueuedConnection);
}
}

int main(int argc, char **argv) 
{
  // init node
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  // Create ros node
  auto node = std::make_shared<rclcpp::Node>("map_gen_live");
  map_gen::MapGenLive map_gen_live(node);

  // Create executor
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  std::thread ros_thread([&exec]() {
    exec.spin();
  });

  map_gen_live.show();
  int ret = app.exec();

  exec.cancel();
  rclcpp::shutdown();
  if (ros_thread.joinable()) {
    ros_thread.join();
  }
  return ret;
}