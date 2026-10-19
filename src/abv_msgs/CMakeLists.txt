cmake_minimum_required(VERSION 3.8)
project(abv_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AbvControlContext.msg"
  "msg/AbvControllerCommand.msg"
  "msg/AbvControllerStatus.msg"
  "msg/AbvGuidanceCommand.msg"
  "msg/AbvGuidanceStatus.msg"
  "msg/AbvHeartbeat.msg"
  "msg/AbvNodeStatus.msg"
  "msg/AbvObstacle.msg"
  "msg/AbvObstacleArray.msg"
  "msg/AbvPath.msg"
  "msg/AbvPolicyActive.msg"
  "msg/AbvPolicyEntry.msg"
  "msg/AbvPolicyRegistry.msg"
  "msg/AbvResponse.msg"
  "msg/AbvState.msg"
  "msg/AbvThrusterCommand.msg"
  "msg/AbvThrusterStatus.msg"
  "msg/AbvVec3.msg"
  "srv/AbvPolicySwitch.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_generators std_msgs)
ament_package()