cmake_minimum_required(VERSION 3.8)
project(light_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 包含头文件目录
include_directories(include)

# 创建共享库
add_library(${PROJECT_NAME} SHARED
  src/heartbeat.cpp
)

# 定义依赖项
set(dependencies
  rclcpp
  std_msgs
)

# 链接依赖库
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装库文件 - 注意EXPORT参数应与后续一致
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}  # 这里修改为统一的导出名称
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 导出目标和依赖，供其他包使用 - 确保与EXPORT参数匹配
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)

# 安装导出配置，使其他包能找到本包 - 使用相同的导出名称
install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE ${PROJECT_NAME}::
)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 生成包配置文件
ament_package()
    