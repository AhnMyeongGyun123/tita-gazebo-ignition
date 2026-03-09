cmake_minimum_required(VERSION 3.8)
project(tita_bal_ign)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(ament_cmake REQUIRED)
set(RMW_IMPLEMENTATION rmw_fastrtps_cpp)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joy REQUIRED)
find_package(gazebo REQUIRED)

find_package(RBDL COMPONENTS URDFREADER REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(
  include
  # /opt/ros/humble/include
  # /opt/ros/humble/include/rclcpp/rclcpp
  # /opt/ros/humble/include/sensor_msgs/msg
  # /usr/local/include/osqp
  ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  /usr/local/lib
  ${GAZEBO_LIBRARY_DIRS}
  )

file(GLOB_RECURSE SRC_PATH CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*)

add_executable(tita_ctr ${SRC_PATH})
ament_target_dependencies(tita_ctr rclcpp std_msgs sensor_msgs joy RBDL Eigen3 )

target_include_directories(tita_ctr PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(tita_ctr

  ${RBDL_LIBRARY}
  ${RBDL_URDFREADER_LIBRARY}
  #/usr/local/lib/librbdl.so
  #/usr/local/lib/librbdl_urdfreader.so
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
  ${joy_LIBRARIES}
  )

set_target_properties(tita_ctr PROPERTIES
  INSTALL_RPATH "/usr/local/lib"
  BUILD_RPATH "/usr/local/lib"
)

install(TARGETS tita_ctr
  DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY
  launch
  config
  model
  world
  DESTINATION share/${PROJECT_NAME})

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

ament_package()
