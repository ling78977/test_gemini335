cmake_minimum_required(VERSION 3.8)
project(test_mapping)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Glog REQUIRED)
find_package(g2o REQUIRED)
find_package(tf2_ros REQUIRED)


set(g2o_libs
  g2o::stuff
  g2o::core

  # ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_cholmod.so
  g2o::solver_dense
  g2o::solver_csparse
  g2o::csparse_extension
  g2o::types_sba
  ${CSPARSE_LIBRARY}
  ${CHOLMOD_LIBRARY}
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/sophus
  ${OpenCV_INCLUDE_DIRS}
  ${g2o_INCLUDE_DIRS}
  /usr/local/include
)

add_executable(${PROJECT_NAME}_node
  src/ndt_inc.cc
  src/test_node.cpp
  
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs message_filters pcl_msgs geometry_msgs)
target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  glog::glog
  ${OpenCV_LIBRARIES}
  ${g2o_libs}
  tf2_ros::tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS

  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package(

  # INSTALL_TO_SHARE

  # config
  # launch
)
