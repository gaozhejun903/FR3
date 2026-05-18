cmake_minimum_required(VERSION 3.8)
project(epg50_gripper_ros)

# 默认使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# 移除对 epg50_gripper_msgs 的依赖
find_package(rosidl_default_generators REQUIRED)

# 添加服务文件列表
set(srv_files
  "srv/GripperCommand.srv"
  "srv/GripperStatus.srv"
  "srv/RenameGripper.srv"
)

# 添加消息文件列表
set(msg_files
  "msg/GripperStatus.msg"
)

# 生成服务和消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# 包含目录
include_directories(include)

# 添加可执行文件
add_executable(epg50_gripper_node src/epg50_gripper_node.cpp)
ament_target_dependencies(epg50_gripper_node
  rclcpp
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(epg50_gripper_node
  ${cpp_typesupport_target}
)

# 安装目标
install(TARGETS
  epg50_gripper_node
  DESTINATION lib/${PROJECT_NAME})

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()