cmake_minimum_required(VERSION 3.5)
project(world_handler)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
)

add_executable(${PROJECT_NAME}_node src/world_handler_node.cpp src/world_handler.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
