cmake_minimum_required(VERSION 3.5)
project(mppi_h)

# parameters
option(USE_OPENMP "USE_OPENMP" ON) # ON / OFF

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(mppi_eval_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

if(USE_OPENMP)
  find_package(OpenMP REQUIRED)
  if(OpenMP_FOUND)
      message(WARNING "OpenMP found. Activate CPU acceleration.")
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  endif()
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(mppi_h_core SHARED src/mppi_h_core.cpp)
add_library(mppi_mode1_core SHARED src/mode1_mppi_3d/mppi_3d_core.cpp)
add_library(mppi_mode2_core SHARED src/mode2_mppi_4d/mppi_4d_core.cpp)

ament_target_dependencies(mppi_h_core rclcpp geometry_msgs visualization_msgs nav_msgs tf2 tf2_geometry_msgs grid_map_core grid_map_ros grid_map_msgs mppi_eval_msgs)
ament_target_dependencies(mppi_mode1_core rclcpp geometry_msgs visualization_msgs nav_msgs tf2 tf2_geometry_msgs grid_map_core grid_map_ros grid_map_msgs mppi_eval_msgs)
ament_target_dependencies(mppi_mode2_core rclcpp geometry_msgs visualization_msgs nav_msgs tf2 tf2_geometry_msgs grid_map_core grid_map_ros grid_map_msgs mppi_eval_msgs)

if(USE_OPENMP)
  if (OPENMP_FOUND)
      if (TARGET OpenMP::OpenMP_CXX)
          target_link_libraries(mppi_mode1_core OpenMP::OpenMP_CXX)
          target_link_libraries(mppi_mode2_core OpenMP::OpenMP_CXX)
      endif()
  endif()
endif()

add_executable(${PROJECT_NAME}_node src/mppi_h_node.cpp src/mppi_h.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp geometry_msgs visualization_msgs nav_msgs tf2 tf2_geometry_msgs grid_map_core grid_map_ros grid_map_msgs mppi_eval_msgs)
target_link_libraries(${PROJECT_NAME}_node
  mppi_h_core
  mppi_mode1_core
  mppi_mode2_core
)

install(TARGETS mppi_h_core mppi_mode1_core mppi_mode2_core ${PROJECT_NAME}_node
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
