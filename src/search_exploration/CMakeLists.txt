cmake_minimum_required(VERSION 3.8)
project(search_exploration)

# WFD, Hungarian and MRTSP run on every map update: build optimised by default.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(search_msgs REQUIRED)

# ROS-free algorithm library.
add_library(search_exploration_core
  src/lib/wfd.cpp
  src/lib/frontier_goal.cpp
  src/lib/hungarian.cpp
  src/lib/mrtsp.cpp
  src/lib/allocation.cpp
)
target_include_directories(search_exploration_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

set(NODES
  frontier_detector_node
  task_allocator_node
  exploration_dispatcher_node
  mission_coordinator_node
)
foreach(node ${NODES})
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/${node}.cpp)
    add_executable(${node} src/${node}.cpp)
    target_link_libraries(${node} search_exploration_core)
    ament_target_dependencies(${node}
      rclcpp rclcpp_action nav_msgs nav2_msgs geometry_msgs visualization_msgs
      std_msgs action_msgs search_msgs)
    install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
  endif()
endforeach()

install(TARGETS search_exploration_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_wfd test/test_wfd.cpp)
  target_link_libraries(test_wfd search_exploration_core)
  ament_add_gtest(test_hungarian test/test_hungarian.cpp)
  target_link_libraries(test_hungarian search_exploration_core)
  ament_add_gtest(test_mrtsp test/test_mrtsp.cpp)
  target_link_libraries(test_mrtsp search_exploration_core)
  ament_add_pytest_test(test_runtime test/test_runtime.py
    TIMEOUT 180
    ENV SEARCH_EXPLORATION_BIN_DIR=${CMAKE_CURRENT_BINARY_DIR})
endif()

ament_package()
