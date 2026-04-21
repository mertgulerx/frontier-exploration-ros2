cmake_minimum_required(VERSION 3.8)
project(frontier_exploration_ros2_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(frontier_exploration_ros2 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)

add_library(${PROJECT_NAME}_core
  src/exploration_panel_logic.cpp
  src/exploration_panel_backend.cpp
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}_core
  frontier_exploration_ros2
  rcl_interfaces
  rclcpp
)

add_library(${PROJECT_NAME} SHARED
  include/frontier_exploration_ros2_rviz/exploration_control_panel.hpp
  src/exploration_control_panel.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  Qt5::Widgets
)
ament_target_dependencies(${PROJECT_NAME}
  frontier_exploration_ros2
  pluginlib
  rcl_interfaces
  rclcpp
  rviz_common
)
pluginlib_export_plugin_description_file(rviz_common rviz_plugin_description.xml)

install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES rviz_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_gtest(test_panel_logic
    test/test_panel_logic.cpp
  )
  target_link_libraries(test_panel_logic
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_panel_logic PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_panel_logic
    frontier_exploration_ros2
    rcl_interfaces
    rclcpp
  )

  ament_add_gtest(test_panel_backend
    test/test_panel_backend.cpp
  )
  target_link_libraries(test_panel_backend
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_panel_backend PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_panel_backend
    frontier_exploration_ros2
    rcl_interfaces
    rclcpp
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core ${PROJECT_NAME})
ament_export_dependencies(
  frontier_exploration_ros2
  pluginlib
  rcl_interfaces
  rclcpp
  rviz_common
)

ament_package()
