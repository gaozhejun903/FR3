cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robo_ctrl REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)  # 添加yaml-cpp依赖
find_package(rosidl_default_generators REQUIRED)  # 添加服务生成器

# 生成自定义服务消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TransformPoint.srv"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Include directories
include_directories(include)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build shared library for components
add_library(${PROJECT_NAME}_components SHARED
  src/eye_hand_calibration_data_collector.cpp
  src/tf_point_query.cpp  # 添加TF点位置查询工具源码
  src/dual_end_tf_collector.cpp  # 添加双臂末端TF收集器源码
)
target_compile_definitions(${PROJECT_NAME}_components
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_srvs
  cv_bridge
  image_transport
  tf2_ros
  tf2
  tf2_geometry_msgs
  OpenCV
  std_msgs
  robo_ctrl
)
target_link_libraries(${PROJECT_NAME}_components "${cpp_typesupport_target}")

# Register component
rclcpp_components_register_nodes(${PROJECT_NAME}_components
  "utils::EyeHandCalibrationDataCollector"
  "utils::TFPointQuery"  # 注册TF点位置查询组件
  "utils::DualEndTfCollector"  # 注册双臂末端TF收集器组件
)

# Create eye_hand_calibration_data_collector_node executable
add_executable(eye_hand_calibration_data_collector_node
  src/eye_hand_calibration_data_collector_node.cpp
)
target_link_libraries(eye_hand_calibration_data_collector_node
  ${PROJECT_NAME}_components
)
ament_target_dependencies(eye_hand_calibration_data_collector_node
  rclcpp
)

# Create static_tf_publisher_node executable
add_executable(static_tf_publisher_node
  src/static_tf_publisher.cpp
)
ament_target_dependencies(static_tf_publisher_node
  rclcpp
  tf2_ros
  tf2
  tf2_geometry_msgs
  yaml_cpp_vendor
)

# Create fake_gripper_tf_publisher_node executable
add_executable(fake_gripper_tf_publisher_node
  src/fake_gripper_tf_publisher.cpp
)
ament_target_dependencies(fake_gripper_tf_publisher_node
  rclcpp
  tf2_ros
  tf2
  tf2_geometry_msgs
)

# Create tf_point_query_node executable
add_executable(tf_point_query_node
  src/tf_point_query_node.cpp
)
target_link_libraries(tf_point_query_node
  ${PROJECT_NAME}_components
)
ament_target_dependencies(tf_point_query_node
  rclcpp
)

# Create dual_end_tf_collector_node executable
add_executable(dual_end_tf_collector_node
  src/dual_end_tf_collector_node.cpp
)
target_link_libraries(dual_end_tf_collector_node
  ${PROJECT_NAME}_components
)
ament_target_dependencies(dual_end_tf_collector_node
  rclcpp
)

# Create keyboard_tcp_controller_node executable
add_executable(keyboard_tcp_controller_node
  src/keyboard_tcp_controller_node.cpp
  src/keyboard_tcp_controller.cpp
)
ament_target_dependencies(keyboard_tcp_controller_node
  rclcpp
  geometry_msgs
  robo_ctrl
  OpenCV
  tf2
  tf2_geometry_msgs
)

# Create opencv_test executable for testing
add_executable(opencv_test
  src/opencv_test.cpp
)
ament_target_dependencies(opencv_test
  OpenCV
)

# Install executables
install(TARGETS
  eye_hand_calibration_data_collector_node
  static_tf_publisher_node
  fake_gripper_tf_publisher_node
  tf_point_query_node  # 添加TF点位置查询节点
  dual_end_tf_collector_node  # 添加双臂末端TF收集器节点
  keyboard_tcp_controller_node  # 添加键盘控制器节点
  opencv_test  # 添加OpenCV测试程序
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/keyboard_ctrl.py
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
