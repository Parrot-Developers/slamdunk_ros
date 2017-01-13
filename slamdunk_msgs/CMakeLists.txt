cmake_minimum_required(VERSION 2.8.12)
project(slamdunk_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  BoolStamped.msg
  Quality.msg
  QualityStamped.msg
  ReconfigureLevels.msg
  ServiceTrigger.msg
  StreamingTypes.msg
  VideoModes.msg
  VideoSources.msg
)

add_service_files(
  DIRECTORY srv
  FILES Save.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
