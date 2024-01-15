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
find_package(std_msgs REQUIRED)
find_package(tachimawari REQUIRED)
find_package(tachimawari_interfaces REQUIRED)

find_package(Protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${Protobuf_VERSION}")

# Find gRPC installation
# Looks for gRPCConfig.cmake file installed by gRPC's cmake installation.
find_package(gRPC CONFIG REQUIRED)
message(STATUS "Using gRPC ${gRPC_VERSION}")

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/action/model/action_name.cpp"
  "src/${PROJECT_NAME}/action/model/action.cpp"  
  "src/${PROJECT_NAME}/action/model/pose.cpp"
  "src/${PROJECT_NAME}/action/node/action_manager.cpp"
  "src/${PROJECT_NAME}/action/node/action_node.cpp"
  "src/${PROJECT_NAME}/action/process/interpolator.cpp"
  "src/${PROJECT_NAME}/action/process/joint_process.cpp"
  "src/${PROJECT_NAME}/config/node/config_node.cpp"
  "src/${PROJECT_NAME}/config/utils/config.cpp"
  "src/${PROJECT_NAME}/config/grpc/config.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_base.cpp"  
  "src/${PROJECT_NAME}/config/grpc/call_data_get_config.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_save_config.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_publish_set_joints.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_publish_set_torques.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_subscribe_current_joints.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_run_action.cpp"
  "src/${PROJECT_NAME}/node/akushon_node.cpp"
  )

target_include_directories(${PROJECT_NAME} PUBLIC  
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  akushon_interfaces
  rclcpp
  rclcpp_action
  std_msgs
  tachimawari
  tachimawari_interfaces
  gRPC)

target_link_libraries(${PROJECT_NAME}
  gRPC::grpc++_reflection
  gRPC::grpc++  
)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin"
  INCLUDES DESTINATION "include")

target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

add_executable(action "src/action_main.cpp")
target_include_directories(action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(action ${PROJECT_NAME})

add_executable(interpolator "src/interpolator_main.cpp")
target_include_directories(interpolator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(interpolator ${PROJECT_NAME})

add_executable(main "src/akushon_main.cpp")
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(main ${PROJECT_NAME})

install(TARGETS
  action
  interpolator
  main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ament_index_cpp
  akushon_interfaces
  rclcpp
  rclcpp_action
  std_msgs
  tachimawari
  tachimawari_interfaces)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_package()
