cmake_minimum_required(VERSION 3.8)
project(frontier_exploration_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/decision_map.cpp
  src/frontier_suppression.cpp
  src/frontier_policy.cpp
  src/frontier_search.cpp
  src/frontier_explorer_core_init.cpp
  src/frontier_explorer_core_maps.cpp
  src/frontier_explorer_core_frontiers.cpp
  src/frontier_explorer_core_suppression.cpp
  src/frontier_explorer_core_dispatch.cpp
  src/mrtsp_ordering.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}_core
  action_msgs
  geometry_msgs
  nav_msgs
)

add_library(${PROJECT_NAME}_node
  src/frontier_explorer_node.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
)
ament_target_dependencies(${PROJECT_NAME}_node
  action_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

add_executable(frontier_explorer
  src/main.cpp
)
target_compile_features(frontier_explorer PUBLIC cxx_std_17)
target_link_libraries(frontier_explorer
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_node
)
ament_target_dependencies(frontier_explorer
  rclcpp
)

if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
  if(ipo_supported)
    set_property(TARGET ${PROJECT_NAME}_core PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    set_property(TARGET ${PROJECT_NAME}_node PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    set_property(TARGET frontier_explorer PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()
endif()

install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_node
    frontier_explorer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frontier_snapshot_and_search
    test/test_frontier_snapshot_and_search.cpp
  )
  target_link_libraries(test_frontier_snapshot_and_search
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_frontier_snapshot_and_search PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_frontier_snapshot_and_search
    action_msgs
    geometry_msgs
    nav_msgs
  )

  ament_add_gtest(test_preemption_flow
    test/test_preemption_flow.cpp
  )
  target_link_libraries(test_preemption_flow
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_preemption_flow PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_preemption_flow
    action_msgs
    geometry_msgs
    nav_msgs
  )

  ament_add_gtest(test_qos_utils
    test/test_qos_utils.cpp
  )
  target_include_directories(test_qos_utils PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_qos_utils
    action_msgs
    geometry_msgs
    nav_msgs
    rclcpp
  )

  ament_add_gtest(test_frontier_suppression
    test/test_frontier_suppression.cpp
  )
  target_link_libraries(test_frontier_suppression
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_frontier_suppression PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_frontier_suppression
    action_msgs
    geometry_msgs
    nav_msgs
  )

  ament_add_gtest(test_decision_map_and_mrtsp
    test/test_decision_map_and_mrtsp.cpp
  )
  target_link_libraries(test_decision_map_and_mrtsp
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_decision_map_and_mrtsp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_decision_map_and_mrtsp
    action_msgs
    geometry_msgs
    nav_msgs
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  action_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

ament_package()
