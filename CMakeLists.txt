cmake_minimum_required(VERSION 3.5)
project(akushon)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(akushon_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tachimawari REQUIRED)
find_package(tachimawari_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/action/model/action_name.cpp"
  "src/${PROJECT_NAME}/action/model/action.cpp"
  "src/${PROJECT_NAME}/action/model/pose.cpp"
  "src/${PROJECT_NAME}/action/node/action_manager.cpp"
  "src/${PROJECT_NAME}/action/node/action_node.cpp"
  "src/${PROJECT_NAME}/action/process/interpolator.cpp"
  "src/${PROJECT_NAME}/action/process/joint_process.cpp"
  "src/${PROJECT_NAME}/node/akushon_node.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  akushon_interfaces
  rclcpp
  rclcpp_action
  tachimawari
  tachimawari_interfaces)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(akushon_main "src/akushon_main.cpp")
target_include_directories(akushon_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(akushon_main ${PROJECT_NAME})

install(TARGETS
  akushon_main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_tests
    "test/action/action_test.cpp" 
    "test/action/interpolator_test.cpp" 
    "test/node/akushon_node_test.cpp"
    )

  target_include_directories(${PROJECT_NAME}_tests PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ament_index_cpp
  akushon_interfaces
  rclcpp
  rclcpp_action
  tachimawari
  tachimawari_interfaces)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_package()
