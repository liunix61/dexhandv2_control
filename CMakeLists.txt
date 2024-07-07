cmake_minimum_required(VERSION 3.8)
project(dexhandv2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Set the path to the Dexhand Connect C++ SDK
set(DC_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sdk/dexhandv2_cpp_client)

# Add the SDK as a subdirectory
add_subdirectory(${DC_SDK_DIR})

# Add the include directories
include_directories(
  ${DC_SDK_DIR}/dexhand_connect/include
  include
  ${ament_cmake_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FirmwareVersion.msg"
  "msg/HardwareDescription.msg"
  "msg/DiscoveredHands.msg"
  "msg/ServoVars.msg"
  "msg/ServoVarsTable.msg"
  "msg/ServoDynamics.msg"
  "msg/ServoStatus.msg"
  "msg/ServoDynamicsTable.msg"
  "srv/Reset.srv"
  DEPENDENCIES std_msgs
)

# Get the message type support target
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Native message node
add_executable(native_messaging src/native_message_node.cpp)
ament_target_dependencies(native_messaging rclcpp std_msgs)
target_link_libraries(native_messaging dexhand_connect ${cpp_typesupport_target})




# Install
install(TARGETS
  native_messaging
  DESTINATION lib/${PROJECT_NAME}
)


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
