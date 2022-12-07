cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)

include_directories(include)

# for interfaces
find_package(rosidl_default_generators REQUIRED)
set(srv_files "srv/ModifyString.srv")
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

# executable for the publisher node 
add_executable(talker app/talker.cpp src/basic_publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs tf2 tf2_ros)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

# executable for the subscriber node 
add_executable(listener app/listener.cpp src/basic_subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})

# executable for testing
set(TESTFILES test/minimal_integration_test.cpp)
ament_add_gtest(my_test ${TESTFILES})
ament_target_dependencies(my_test rclcpp std_msgs geometry_msgs rclcpp tf2_ros tf2)

# # executable for recording rosbag file(s)
# add_executable(recorder app/recorder.cpp src/basic_recorder.cpp)
# ament_target_dependencies(recorder rclcpp std_msgs rosbag2_cpp)
# install(TARGETS recorder DESTINATION lib/${PROJECT_NAME})

# link target interfaces
ament_export_dependencies(rosidl_default_runtime)
rosidl_target_interfaces(talker
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(my_test ${PROJECT_NAME} "rosidl_typesupport_cpp")

# launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
