cmake_minimum_required(VERSION 3.8)
project(light_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(light_interfaces REQUIRED)

# 安装Python模块（移除不支持的PATH_SUFFIXES参数）
ament_python_install_package(${PROJECT_NAME})

# 安装可执行脚本
install(PROGRAMS
  light_detector/detector_node.py  # 节点代码的实际路径
  DESTINATION lib/${PROJECT_NAME}
  RENAME detector_node  # 重命名为无.py后缀的可执行文件
)

# 环境钩子配置
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()  # 修复测试函数调用
endif()

ament_package()