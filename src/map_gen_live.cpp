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
    node_->declare_parameter<std::string>("input_topic", "");
    node_->declare_parameter<double>("grid_resolution", 1.0);
    node_->declare_parameter<double>("grid_size_x", 10.0);
    node_->declare_parameter<double>("grid_size_y", 10.0);
    node_->declare_parameter<bool>("use_tf", true);
    std::vector<std::string> param_names = {"input_topic", 
                                            "grid_resolution",
                                            "grid_size_x",
                                            "grid_size_y",
                                            "use_tf"};
    std::vector<rclcpp::Parameter> params_ = node_->get_parameters(param_names);
    for (const auto& param : params_) {
        param_map_[param.get_name()] = param;
    }

    pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        param_map_["input_topic"].as_string(),
        10,
        std::bind(&MapGenLive::pointcloud_cb, this, std::placeholders::_1));

    // Qt and VTK setup
    resize(800, 600);
    setWindowTitle("MapGen Live");
    QToolBar *toolbar = new QToolBar(this);
    addToolBar(toolbar);
    QPushButton *pause_button = new QPushButton("Pause", this);
    toolbar->addWidget(pause_button);
    vtk_widget_ = new QVTKOpenGLNativeWidget(this);
    setCentralWidget(vtk_widget_);
    vtkNew<vtkGenericOpenGLRenderWindow> render_window;
    vtk_widget_->setRenderWindow(render_window);
    vtkNew<vtkRenderer> renderer;
    render_window->AddRenderer(renderer);

    connect(pause_button, &QPushButton::clicked, this, [this, pause_button]() {
        is_paused_ = !is_paused_;
        pause_button->setText(is_paused_ ? "Resume" : "Pause");
    });

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

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(surface_mesh);
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
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
          "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_ONCE(node_->get_logger(), "Could not transform point cloud: %s", ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 transformed_cloud;
    tf2::doTransform(*msg, transformed_cloud, transform_stamped);

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

    for (int y = 0; y < ny_; ++y) {
      for (int x = 0; x < nx_; ++x) {
        double point[3];
        int idx = y * nx_ + x;
        points_->GetPoint(idx, point);
        point[2] = z_averages[idx];
        points_->SetPoint(idx, point);
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