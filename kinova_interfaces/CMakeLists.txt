cmake_minimum_required(VERSION 3.8)
project(kinova_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetObjectCoordinates.srv"
  "srv/GetRobotParameters.srv"
  "srv/GetRelativeMovement.srv"
  "srv/ExecuteRecipe.srv"
  "msg/ExtendedStatus.msg"
  "msg/SystemSummary.msg"
  DEPENDENCIES geometry_msgs
)

install(DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/kinova_interfaces
  DESTINATION lib/python3.10/site-packages/
)

ament_package()