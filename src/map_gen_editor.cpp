#include "map_gen/map_gen_editor.hpp"

namespace map_gen {

MapGenEditor::MapGenEditor(rclcpp::Node::SharedPtr node,
                       const std::string &ns,
                       std::vector<rclcpp::Parameter> parameters,
                       QWidget *parent)
: QMainWindow(parent),
  node_(node),
  nx_(1),
  ny_(1)
  {

  // Declare parameters
  node_->declare_parameter<std::vector<std::string>>("terrain_types", std::vector<std::string>{"soil", "bedrock", "sand", "large_rock"});
  node_->declare_parameter<float>("grid_resolution", 0.1);
  std::vector<std::string> param_names = {"terrain_types", "grid_resolution"};
  std::vector<rclcpp::Parameter> params_ = node_->get_parameters(param_names);
  for (const auto& param : params_) {
      param_map_[param.get_name()] = param;
  }

  // Qt and VTK setup
  resize(QGuiApplication::primaryScreen()->availableGeometry().size());
  setWindowTitle("MapGen Editor");
  QWidget *central_widget = new QWidget(this);
  setCentralWidget(central_widget);
  QHBoxLayout *main_layout = new QHBoxLayout(central_widget);
  main_layout->setContentsMargins(0, 0, 0, 0);
  vtk_widget_ = new QVTKOpenGLNativeWidget(central_widget);
  main_layout->addWidget(vtk_widget_, 1);
  QVBoxLayout *right_layout = new QVBoxLayout();
  main_layout->addLayout(right_layout);
  vtk_widget_2d_ = new QVTKOpenGLNativeWidget(central_widget);
  vtk_widget_2d_->setFixedSize(200, 200);
  right_layout->addWidget(vtk_widget_2d_);
  right_layout->addStretch();
  vtkNew<vtkGenericOpenGLRenderWindow> render_window;
  vtk_widget_->setRenderWindow(render_window);
  vtkNew<vtkRenderer> renderer;
  render_window->AddRenderer(renderer);

  // Toolbar setup
  QToolBar *toolbar = new QToolBar(this);
  addToolBar(toolbar);
  QPushButton *tool_button = new QPushButton("Tool", this);
  toolbar->addWidget(tool_button);
  connect(tool_button, &QPushButton::clicked, this, [this, tool_button]() {
    tool_enabled_ = !tool_enabled_;
    tool_button->setText(tool_enabled_ ? "Stop Tool" : "Tool"); 
  });
  QComboBox *tool_dropdown = new QComboBox(this);
  for (std::string val : terrain_tool_avail_)
  {
    tool_dropdown->addItem(QString::fromStdString(val));
  }
    for (std::string val : geom_tool_avail_)
  {
    tool_dropdown->addItem(QString::fromStdString(val));
  }
  toolbar->addWidget(tool_dropdown);
  connect(tool_dropdown, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
    tool_mode_ = index;
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
  QLabel *brush_label = new QLabel("Brush Size:", this);
  toolbar->addWidget(brush_label);
  QSpinBox *brush_size_spinbox = new QSpinBox(this);
  brush_size_spinbox->setRange(0, 1000);
  brush_size_spinbox->setValue(brush_size_);
  toolbar->addWidget(brush_size_spinbox);
  connect(brush_size_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
    brush_size_ = value;
  });
  QLabel *intensity_label = new QLabel("Brush Intensity:", this);
  toolbar->addWidget(intensity_label);
  QDoubleSpinBox *intensity_spinbox = new QDoubleSpinBox(this);
  intensity_spinbox->setRange(0.0, 1.0);
  intensity_spinbox->setDecimals(3);
  intensity_spinbox->setSingleStep(0.001);
  intensity_spinbox->setValue(brush_intensity_);
  toolbar->addWidget(intensity_spinbox);
  connect(intensity_spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
    brush_intensity_ = value;
  });
  
  QWidget *spacer = new QWidget(this);
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  toolbar->addWidget(spacer);
  QPushButton *open_button = new QPushButton("Open", this);
  toolbar->addWidget(open_button);
  connect(open_button, &QPushButton::clicked, this, &MapGenEditor::open_cb);
  QPushButton *open_terrain_button = new QPushButton("Open Terrain", this);
  toolbar->addWidget(open_terrain_button);
  connect(open_terrain_button, &QPushButton::clicked, this, &MapGenEditor::open_terrain_cb);
  QPushButton *save_button = new QPushButton("Save", this);
  toolbar->addWidget(save_button);
  connect(save_button, &QPushButton::clicked, this, &MapGenEditor::save_cb);
  QPushButton *clear_button = new QPushButton("Clear", this);
  toolbar->addWidget(clear_button);
  connect(clear_button, &QPushButton::clicked, this, &MapGenEditor::clear_cb);

  // Color map setup
  vtkNew<vtkLookupTable> lut;
  int num_terrains = terrain_types.size();
  lut->SetNumberOfTableValues(num_terrains);
  lut->SetTableRange(0, std::max(1, num_terrains - 1));
  lut->SetBelowRangeColor(0.2, 0.2, 0.2, 1.0);
  lut->UseBelowRangeColorOn();
  
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

  // Keyboard Shortcuts
  QShortcut *disable_shortcut = new QShortcut(QKeySequence(Qt::Key_Escape), this);
  connect(disable_shortcut, &QShortcut::activated, this, [this, tool_button]() {
    tool_enabled_ = false;
    tool_button->setText(tool_enabled_ ? "Stop Tool" : "Tool");
  });

  // Initialize terrain map and surface mesh
  surface_mesh_ = vtkSmartPointer<vtkPolyData>::New();
  terrain_map_ = vtkSmartPointer<vtkIntArray>::New();
  terrain_map_->SetNumberOfValues(1);
  terrain_map_->SetValue(0, 0);

  // Mouse interaction setup
  vtkNew<vtkCallbackCommand> click_callback;
  click_callback->SetClientData(this);
  click_callback->SetCallback(MapGenEditor::click_cb);
  auto* interactor_style = render_window->GetInteractor()->GetInteractorStyle();
  interactor_style->RemoveObservers(vtkCommand::LeftButtonPressEvent);
  interactor_style->RemoveObservers(vtkCommand::LeftButtonReleaseEvent);
  interactor_style->RemoveObservers(vtkCommand::MouseMoveEvent);
  interactor_style->AddObserver(vtkCommand::LeftButtonPressEvent, click_callback);
  interactor_style->AddObserver(vtkCommand::LeftButtonReleaseEvent, click_callback);
  interactor_style->AddObserver(vtkCommand::MouseMoveEvent, click_callback);

  // Create mapper and actor
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(surface_mesh_);
  mapper->SetScalarRange(0, std::max(1, num_terrains - 1));
  mapper->SetLookupTable(lut);
  mapper->SetScalarModeToUsePointData();
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  renderer->AddActor(actor);
  renderer->ResetCamera();

  // 2D view setup
  vtkNew<vtkGenericOpenGLRenderWindow> render_window_2d;
  vtk_widget_2d_->setRenderWindow(render_window_2d);
  vtkNew<vtkRenderer> renderer_2d;
  render_window_2d->AddRenderer(renderer_2d);
  image_data_ = vtkSmartPointer<vtkImageData>::New();
  image_data_->SetDimensions(nx_, ny_, 1);
  image_data_->GetPointData()->SetScalars(terrain_map_);
  vtkNew<vtkImageMapToColors> color_mapper;
  color_mapper->SetInputData(image_data_);
  color_mapper->SetLookupTable(lut);
  image_actor_ = vtkSmartPointer<vtkImageActor>::New();
  image_actor_->GetMapper()->SetInputConnection(color_mapper->GetOutputPort());
  renderer_2d->AddActor(image_actor_);
  renderer_2d->ResetCamera();
  vtkNew<vtkInteractorStyle> style_2d;
  render_window_2d->GetInteractor()->SetInteractorStyle(style_2d);
}

void MapGenEditor::click_cb(vtkObject* caller, long unsigned int event_id, void* client_data, void* call_data)
{
  auto* self = static_cast<MapGenEditor*>(client_data);
  auto* style = vtkInteractorStyle::SafeDownCast(caller);
  auto* interactor = style->GetInteractor();

  bool handled = false;
  if (self->tool_enabled_) {
    if (event_id == vtkCommand::LeftButtonPressEvent && !interactor->GetShiftKey()) {
      self->is_mousedown_ = true;
      handled = true;
    }
    else if (event_id == vtkCommand::LeftButtonReleaseEvent && self->is_mousedown_) {
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
      std::vector<int> picked_points;
      if (point_id != -1) {
        int cx = point_id % self->nx_;
        int cy = point_id / self->nx_;
        int r = self->brush_size_;
        for (int dy = -r; dy <= r; ++dy) {
          for (int dx = -r; dx <= r; ++dx) {
            if (dx * dx + dy * dy <= r * r) {
              int nx = cx + dx;
              int ny = cy + dy;
              if (nx >= 0 && nx < self->nx_ && ny >= 0 && ny < self->ny_) {
                int idx = ny * self->nx_ + nx;
                picked_points.push_back(idx);
              }
            }
          }
        }
        if (self->tool_mode_ < self->terrain_tool_avail_.size())
        {
          self->edit_terrain(picked_points);
        }
        else
        {
          self->edit_geometry(picked_points);
        } 

        self->terrain_map_->Modified();
        self->image_data_->Modified();
        interactor->GetRenderWindow()->Render();
        self->vtk_widget_2d_->renderWindow()->Render();
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
}

void MapGenEditor::edit_terrain(std::vector<int> indices)
{
  for (int idx : indices)
  {
    auto terrain_value = terrain_map_->GetValue(idx);
    if (tool_mode_ == 0 && terrain_value != -1) { // draw 
      terrain_map_->SetValue(idx, draw_type_);
    }
    else if (tool_mode_ == 1 && terrain_value != -1) { // erase
      terrain_map_->SetValue(idx, 0);
    }
    else if (tool_mode_ == 2) { // block
      terrain_map_->SetValue(idx, -1);
    }
    else if (tool_mode_ == 3) { // unblock
      terrain_map_->SetValue(idx, 0); // somehow feels more intuitive to set 0 here instead of actual selected terrain type
    }
  }
}

void MapGenEditor::edit_geometry(std::vector<int> indices)
{
  vtkPoints* points = surface_mesh_->GetPoints();
  size_t geom_mode = tool_mode_ - terrain_tool_avail_.size(); 
  
  if (geom_mode == 0) { // push
    for (int idx : indices) {
      double pt[3];
      points->GetPoint(idx, pt);
      pt[2] -= brush_intensity_;
      points->SetPoint(idx, pt);
    }
  }
  else if (geom_mode == 1) { // pull
    for (int idx : indices) {
      double pt[3];
      points->GetPoint(idx, pt);
      pt[2] += brush_intensity_;
      points->SetPoint(idx, pt);
    }
  }
  else if (geom_mode == 2) { // flatten
    std::vector<double> new_z(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
      int idx = indices[i];
      bool is_blocked = terrain_map_->GetValue(idx) == -1;
      double pt[3];
      points->GetPoint(idx, pt);
      int cx = idx % nx_;
      int cy = idx / nx_;
      double sum_z = 0.0;
      int count = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        int ny = cy + dy;
        if (ny < 0 || ny >= ny_) continue;
        for(int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;
          int nx = cx + dx;
          if (nx < 0 || nx >= nx_) continue;
          int n_idx = ny * nx_ + nx;
          // blocked vertices consider blocked neighbours otherwise they never move
          if (is_blocked || terrain_map_->GetValue(n_idx) != -1) {
            double n_pt[3];
            points->GetPoint(n_idx, n_pt);
            sum_z += n_pt[2];
            ++count;
          }
        }
      }
      if (count > 0) {
        new_z[i] = pt[2] + ((sum_z / count) - pt[2]) * brush_intensity_;
      }
      else {
        new_z[i] = pt[2];
      }
    }
    for (size_t i = 0; i < indices.size(); ++i) {
      double pt[3];
      points->GetPoint(indices[i], pt);
      pt[2] = new_z[i];
      points->SetPoint(indices[i], pt);
    }
  }
  else if (geom_mode == 3) { // get_coord
    if (!indices.empty()) {
      double pt[3];
      points->GetPoint(indices[0], pt);
      QMessageBox::information(this, "Coordinate", QString("X: %1, Y: %2, Z: %3").arg(pt[0]).arg(pt[1]).arg(pt[2]));
      is_mousedown_ = false;
    }
  }
  points->Modified();
  surface_mesh_->Modified();
}

void MapGenEditor::open_cb()
{
  // File Dialog
  QString file_path = QFileDialog::getOpenFileName(this, "Open Mesh File", "", "Mesh Files (*.obj *.stl);;All Files (*)");
  if (file_path.isEmpty()) {
    return;
  }

  // Parse file
  vtkSmartPointer<vtkPolyData> loaded_mesh;
  if (file_path.endsWith(".obj", Qt::CaseInsensitive)) {
    vtkNew<vtkOBJReader> reader;
    reader->SetFileName(file_path.toStdString().c_str());
    reader->Update();
    loaded_mesh = reader->GetOutput();
  }
  else if (file_path.endsWith(".stl", Qt::CaseInsensitive)) {
    vtkNew<vtkSTLReader> reader;
    reader->SetFileName(file_path.toStdString().c_str());
    reader->Update();
    loaded_mesh = reader->GetOutput();
  } 
  else {
    QMessageBox::warning(this, "Unsupported File", "The selected file format is not supported.");
    return;
  }
  vtkIdType num_points = loaded_mesh->GetNumberOfPoints();
  if (num_points == 0) {
    return;
  }

  // Sample at fixed distance to make it square shaped
  double res = param_map_["grid_resolution"].as_double();
  double bounds[6];
  loaded_mesh->GetBounds(bounds);
  double max_dim = std::max({bounds[1] - bounds[0], bounds[3] - bounds[2]});
  int n = static_cast<int>(std::ceil(max_dim / res)) + 1;
  nx_ = n;
  ny_ = n;

  vtkNew<vtkCellLocator> locator;
  locator->SetDataSet(loaded_mesh);
  locator->BuildLocator();

  vtkNew<vtkPoints> points;
  points->SetNumberOfPoints(nx_ * ny_);
  terrain_map_->SetNumberOfValues(nx_ * ny_);
  double z_max = bounds[5] + 1.0;
  double z_min = bounds[4] - 1.0;
  double x_min = bounds[0];
  double y_min = bounds[2];

  for (int y = 0; y < ny_; ++y) {
    double py = y_min + y * res;
    for (int x = 0; x < nx_; ++x) {
      double px = x_min + x * res;
      int idx = y * nx_ + x;

      double p1[3] = {px, py, z_max};
      double p2[3] = {px, py, z_min};
      double t = 0.0;
      double hit[3] = {0.0, 0.0, 0.0};
      double pcoords[3] = {0.0, 0.0, 0.0};
      int sub_id = 0;
      vtkIdType cell_id = -1;

      if (locator->IntersectWithLine(p1, p2, 0.001, t, hit, pcoords, sub_id, cell_id)) {
        points->SetPoint(idx, px - x_min, py - y_min, hit[2]);
        terrain_map_->SetValue(idx, 0); // Default terrain type
      } else {
        points->SetPoint(idx, px - x_min, py - y_min, z_min);
        terrain_map_->SetValue(idx, -1); // Blocked
      }
    }
  }
  terrain_map_->Modified();

  // change border vertices to be the same height as surrounding to avoid the robot leaving the map
  for (int y = 0; y < ny_; ++y) {
    double py = y_min + y * res;
    for (int x = 0; x < nx_; ++x) {
      int idx = y * nx_ + x;
      if (terrain_map_->GetValue(idx) == -1) {
        double px = x_min + x * res;
        double sum_z = 0.0;
        int count = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          int ny = y + dy;
          if (ny < 0 || ny >= ny_) continue;
          for (int dx = -1; dx <= 1; ++dx) {
            int nx = x + dx;
            if (nx < 0 || nx >= nx_) continue;
            int n_idx = ny * nx_ + nx;
            if (terrain_map_->GetValue(n_idx) != -1) {
              sum_z += points->GetPoint(n_idx)[2];
              ++count;
            }
          }
        }
        if (count > 0) {
          points->SetPoint(idx, px - x_min, py - y_min, sum_z / count);
        }
      }
    }
  }

  // generate triangles
  vtkNew<vtkCellArray> polys;
  for (int y = 0; y < ny_ - 1; ++y) {
    for (int x = 0; x < nx_ - 1; ++x) {
      vtkIdType p0 = y * nx_ + x;
      vtkIdType p1 = p0 + 1;
      vtkIdType p2 = p0 + nx_;
      vtkIdType p3 = p2 + 1;
      vtkIdType cell_a[3] = {p0, p1, p2};
      polys->InsertNextCell(3, cell_a);
      vtkIdType cell_b[3] = {p1, p3, p2};
      polys->InsertNextCell(3, cell_b);
    }
  }

  // update mesh and render
  surface_mesh_->SetPoints(points);
  surface_mesh_->SetPolys(polys);
  surface_mesh_->GetPointData()->SetScalars(terrain_map_);
  surface_mesh_->Modified();

  image_data_->SetDimensions(nx_, ny_, 1);
  image_data_->GetPointData()->SetScalars(terrain_map_);
  image_data_->Modified();
  image_actor_->SetDisplayExtent(0, nx_ - 1, 0, ny_ - 1, 0, 0);

  vtk_widget_->renderWindow()->GetRenderers()->GetFirstRenderer()->ResetCamera();
  vtk_widget_->renderWindow()->Render();
  vtk_widget_2d_->renderWindow()->GetRenderers()->GetFirstRenderer()->ResetCamera();
  vtk_widget_2d_->renderWindow()->Render();
}

void MapGenEditor::open_terrain_cb()
{
  QString file_path = QFileDialog::getOpenFileName(this, "Open Terrain YAML", "", "YAML Files (*.yaml *.yml);;All Files (*)");
  if (file_path.isEmpty()) {
    return;
  }

  std::ifstream file(file_path.toStdString());
  if (!file.is_open()) {
    QMessageBox::warning(this, "Error", "Failed to open terrain YAML file.");
    return;
  }

  std::vector<std::vector<int>> loaded_grid;
  std::string line;
  bool in_terrain = false;

  while (std::getline(file, line)) {
    size_t start = line.find_first_not_of(" \t\r\n");
    if (start == std::string::npos || line[start] == '#') {
      continue;
    }

    if (line.find("terrain:") != std::string::npos) {
      in_terrain = true;
      continue;
    }

    if (in_terrain) {
      size_t lbracket = line.find('[');
      size_t rbracket = line.find(']');
      if (lbracket != std::string::npos && rbracket != std::string::npos && rbracket > lbracket) {
        std::string row_str = line.substr(lbracket + 1, rbracket - lbracket - 1);
        std::vector<int> row;
        std::stringstream ss(row_str);
        std::string item;
        while (std::getline(ss, item, ',')) {
          try {
            row.push_back(std::stoi(item));
          } catch (...) {
            row.push_back(0);
          }
        }
        if (!row.empty()) {
          loaded_grid.push_back(row);
        }
      } else if (line[start] != '-') {
        break;
      }
    }
  }

  if (loaded_grid.empty() || loaded_grid[0].empty()) {
    QMessageBox::warning(this, "Invalid File", "No valid terrain grid found in YAML.");
    return;
  }

  int src_ny = static_cast<int>(loaded_grid.size());
  int src_nx = static_cast<int>(loaded_grid[0].size());

  for (int y = 0; y < ny_; ++y) {
    int src_y = (ny_ > 1) ? static_cast<int>(std::round(static_cast<double>(y) * (src_ny - 1) / (ny_ - 1))) : 0;
    src_y = std::clamp(src_y, 0, src_ny - 1);

    for (int x = 0; x < nx_; ++x) {
      int idx = y * nx_ + x;
      int src_x = (nx_ > 1) ? static_cast<int>(std::round(static_cast<double>(x) * (src_nx - 1) / (nx_ - 1))) : 0;
      src_x = std::clamp(src_x, 0, static_cast<int>(loaded_grid[src_y].size()) - 1);

      terrain_map_->SetValue(idx, loaded_grid[src_y][src_x]);
    }
  }

  terrain_map_->Modified();
  surface_mesh_->Modified();
  image_data_->Modified();

  vtk_widget_->renderWindow()->Render();
  vtk_widget_2d_->renderWindow()->Render();
}

void MapGenEditor::save_cb()
{
  QString dir = QFileDialog::getExistingDirectory(this, "Select Directory to Save Output Files",
                                                  "",
                                                  QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  
  if (dir.isEmpty()) {
    return; // User canceled the dialog
  }

  std::string path = dir.toStdString() + "/";

  vtkNew<vtkSTLWriter> stl_writer;
  stl_writer->SetFileName((path +"terrain_mesh.stl").c_str());
  stl_writer->SetInputData(surface_mesh_);
  stl_writer->Write();

  std::ofstream yaml_file(path + "terrain_map.yaml");
  if (yaml_file.is_open()) {
    yaml_file << "terrain:\n";
    for (int y = 0; y < ny_; ++y) {
      yaml_file << "  - [";
      for (int x = 0; x < nx_; ++x) {
        int val = terrain_map_->GetValue(y * nx_ + x);
        yaml_file << val << (x < nx_ - 1 ? ", " : "");
      }
      yaml_file << "]\n";
    }
    yaml_file.close();
  }

  vtkNew<vtkWindowToImageFilter> window_to_image;
  window_to_image->SetInput(vtk_widget_2d_->renderWindow());
  vtk_widget_2d_->renderWindow()->Render();
  window_to_image->Update();
  vtkNew<vtkPNGWriter> png_writer;
  png_writer->SetFileName((path + "terrain_map.png").c_str());
  png_writer->SetInputConnection(window_to_image->GetOutputPort());
  png_writer->Write();
}

void MapGenEditor::clear_cb()
{
  for (int i = 0; i < nx_ * ny_; ++i) {
    if (terrain_map_->GetValue(i) != -1) // Don't clear blocked cells
    {
      terrain_map_->SetValue(i, 0);
    }
  }
  terrain_map_->Modified();
  image_data_->Modified();
  vtk_widget_->renderWindow()->Render();
  vtk_widget_2d_->renderWindow()->Render();
}

} // namespace map_gen

int main(int argc, char **argv) 
{
  // init node
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  // Create ros node
  auto node = std::make_shared<rclcpp::Node>("map_gen_live");
  map_gen::MapGenEditor map_gen_live(node);

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