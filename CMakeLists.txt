cmake_minimum_required(VERSION 3.5)
project(nav_msgs_custom)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_java REQUIRED)

set(ROSIDL_GENERATOR_LANGUAGES "rosidl_generator_java")  # ✅ set this explicitly

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GPSWaypointArray.msg"
  DEPENDENCIES std_msgs
)

ament_package()
