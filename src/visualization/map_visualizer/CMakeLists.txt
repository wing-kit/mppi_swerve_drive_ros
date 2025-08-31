cmake_minimum_required(VERSION 3.5)
project(map_visualizer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

add_library(map_visualizer SHARED src/map_visualizer.cpp)
ament_target_dependencies(map_visualizer rclcpp visualization_msgs tf2 tf2_geometry_msgs)

add_executable(${PROJECT_NAME}_node src/map_visualizer_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)
target_link_libraries(${PROJECT_NAME}_node map_visualizer)

install(TARGETS map_visualizer ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
