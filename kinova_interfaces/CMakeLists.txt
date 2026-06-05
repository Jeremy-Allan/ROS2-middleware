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
  "srv/MoveArm.srv"
  "srv/MoveGripper.srv"
  "srv/RelativeMove.srv"
  "srv/HomeArm.srv"
  "msg/ExtendedStatus.msg"
  "msg/SystemSummary.msg"
  DEPENDENCIES geometry_msgs
)

ament_package()