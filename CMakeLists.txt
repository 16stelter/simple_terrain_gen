cmake_minimum_required(VERSION 3.12)
project(map_gen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()  

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Qt6)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fastnoiselite REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(VTK REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GenerateMap.srv"
  "srv/GenerateTerrain.srv"
  DEPENDENCIES 
  sensor_msgs
)

include_directories(include 
                    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp 
                    ${CGAL_INCLUDE_DIRS}
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
add_compile_options(-Wall -Werror -Wno-unused -fPIC)

add_executable(MapGen src/map_gen.cpp)
add_executable(MapGenTest src/map_gen_test.cpp)
add_executable(MapGenEditor src/map_gen_editor.cpp)
add_executable(TerrainGen src/terrain_gen.cpp)

set(COMMON_DEPS
    ament_cmake
    rclcpp
    rosidl_default_runtime
    ament_index_cpp
)

ament_target_dependencies(
  MapGen
  ${COMMON_DEPS}
)

ament_target_dependencies(
  MapGenTest
  ${COMMON_DEPS}
)

ament_target_dependencies(
  MapGenEditor
  ${COMMON_DEPS}
  yaml-cpp
)

ament_target_dependencies(
  TerrainGen
  ${COMMON_DEPS}
  fastnoiselite
  OpenCV
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

set(CGAL_LIBS
    CGAL::CGAL
    Qt6::Core
    Qt6::Widgets
    CGAL::CGAL_Basic_viewer
)

target_link_libraries(MapGen
    "${cpp_typesupport_target}"
    rclcpp::rclcpp
    ${CGAL_LIBS}
)

target_link_libraries(MapGenTest
    "${cpp_typesupport_target}"
    rclcpp::rclcpp
    ${CGAL_LIBS}
)

target_link_libraries(MapGenEditor
    "${cpp_typesupport_target}"    
    rclcpp::rclcpp
    yaml-cpp::yaml-cpp
    ${VTK_LIBRARIES}
    Qt6::Core
    Qt6::Widgets
)

target_link_libraries(TerrainGen
    "${cpp_typesupport_target}"
    rclcpp::rclcpp
    yaml-cpp::yaml-cpp  
)

install(
    TARGETS
        MapGen
        MapGenTest
        MapGenEditor
        TerrainGen
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY
        config
        launch
    DESTINATION share/${PROJECT_NAME}
)
ament_package()
