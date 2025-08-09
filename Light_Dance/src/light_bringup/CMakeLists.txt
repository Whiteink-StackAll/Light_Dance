cmake_minimum_required(VERSION 3.8)
project(light_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 2 基础依赖
find_package(ament_cmake REQUIRED)

# 无需编译代码，仅安装资源文件
# 安装 launch 目录
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 config 目录
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 导出与打包
ament_package()