cmake_minimum_required(VERSION 3.8)
project(light_solver)

# 启用 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(light_interfaces REQUIRED)
find_package(light_utils REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)  # 添加cv_bridge依赖
find_package(Eigen3 3.3 REQUIRED NO_MODULE)  # 明确Eigen版本

# 包含头文件路径
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}  # 添加cv_bridge头文件路径
)

# 声明库目标
add_library(
  ${PROJECT_NAME}_component SHARED
  src/one_euro_filter.cpp
  src/solver_method.cpp
  src/solver_node.cpp
)

# 编译选项
target_compile_options(${PROJECT_NAME}_component PRIVATE 
  -Wall 
  -Wextra 
  -Wpedantic 
)

# 链接依赖库
ament_target_dependencies(
  ${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  light_interfaces
  light_utils
  cv_bridge  # 添加cv_bridge到依赖项
)

# 显式链接Eigen和OpenCV
target_link_libraries(${PROJECT_NAME}_component
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}  # 链接cv_bridge库
)

# 注册节点
rclcpp_components_register_nodes(${PROJECT_NAME}_component "light_solver::SolverNode")

# 安装
install(
  TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
