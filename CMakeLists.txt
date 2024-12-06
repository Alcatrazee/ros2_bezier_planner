cmake_minimum_required(VERSION 3.8)
project(bezier_curve_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)

include_directories(include)
# 定义库名称
set(library_name ${PROJECT_NAME}_plugin)
# 创建共享库
add_library(${library_name} SHARED  src/bezier_curve_planner.cpp src/bezier_curve_generator.cpp)
# 指定库的依赖关系
ament_target_dependencies(${library_name} nav2_core pluginlib)
# 安装库文件到指定目录
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# 安装头文件到指定目录
install(DIRECTORY include/
  DESTINATION include/ )
pluginlib_export_plugin_description_file(nav2_core bezier_curve_planner_plugin.xml)

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
