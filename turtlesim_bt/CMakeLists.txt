cmake_minimum_required(VERSION 3.8)
project(turtlesim_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED) 
find_package(turtlesim REQUIRED)
find_package(ament_index_cpp REQUIRED)


add_executable(turtle_bt src/turtle_bt.cpp)

# Proper ament linking
ament_target_dependencies(turtle_bt
  rclcpp
  geometry_msgs 
  behaviortree_cpp_v3
  turtlesim
  ament_index_cpp
)

# Manually link zmq
target_link_libraries(turtle_bt zmq)


install(TARGETS
  turtle_bt
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
