cmake_minimum_required(VERSION 3.8)
project(light_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找 ROS 2 依赖（仅保留消息生成必需的依赖）
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # SendData.msg 依赖 std_msgs/Header
find_package(geometry_msgs REQUIRED)  # 如果你有消息依赖 geometry_msgs

# 2. 声明消息/服务文件（确保路径和文件存在）
set(msg_files
  "msg/SendData.msg"
  "msg/Armor.msg"
)

set(srv_files
  "srv/GetAimState.srv"  # 服务文件单独列出，更清晰
)

# 3. 生成消息和服务接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs  # 依赖的接口包
)

# 4. 安装与导出
ament_export_dependencies(rosidl_default_runtime)
ament_package()