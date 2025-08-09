cmake_minimum_required(VERSION 3.8)
project(light_serial)

# 明确C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 降低优化级别以减少编译器内存使用
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 确保生成器被正确设置
if(NOT CMAKE_GENERATOR)
  if(UNIX)
    set(CMAKE_GENERATOR "Unix Makefiles" CACHE INTERNAL "" FORCE)
  endif()
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(light_interfaces REQUIRED)  
find_package(light_utils REQUIRED)

# 验证所有依赖是否找到
if(NOT rclcpp_FOUND)
  message(FATAL_ERROR "rclcpp package not found")
endif()

if(NOT light_interfaces_FOUND)
  message(FATAL_ERROR "light_interfaces package not found")
endif()

if(NOT light_utils_FOUND)
  message(FATAL_ERROR "light_utils package not found")
endif()

# 添加库目标（共享库，用于组件化）
add_library(
  ${PROJECT_NAME}_lib
  SHARED
  src/uart_driver.cpp
  src/uart_node.cpp
  src/uart_protocol.cpp
)

# 设置头文件包含路径
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 链接依赖的ROS 2库
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  geometry_msgs
  light_interfaces
  light_utils
)

# 链接系统库
target_link_libraries(${PROJECT_NAME}_lib
  pthread
  rt
)

# 注册组件节点
rclcpp_components_register_nodes(${PROJECT_NAME}_lib "light_serial::UARTNode")

# 安装库
install(TARGETS
  ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY
  include/
  DESTINATION include
)

# 导出依赖
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp 
  rclcpp_components 
  geometry_msgs 
  light_interfaces 
  light_utils
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
    