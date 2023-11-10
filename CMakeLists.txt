cmake_minimum_required(VERSION 3.0.2)
project(motor_response)

add_compile_options(-std=c++11)

find_package(
  catkin
  REQUIRED COMPONENTS
    control_msgs
    message_generation
    control_toolbox
    roscpp
    urdf
)

add_message_files(
  DIRECTORY msg
  FILES
  Adc.msg
  Pwm.msg
  PwmChannel.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    actionlib
    control_msgs
    control_toolbox
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${motor_response_INCLUDE_DIRS}
)

add_executable(
  sample
  src/sample.cpp
)

add_executable(
  tune
  src/tune.cpp
)

add_dependencies(
  sample
  ${catkin_EXPORTED_TARGETS}
  ${motor_response_EXPORTED_TARGETS}
)

add_dependencies(
  tune
  ${catkin_EXPORTED_TARGETS}
  ${motor_response_EXPORTED_TARGETS}
)

target_link_libraries(
  sample
  ${catkin_LIBRARIES}
)

target_link_libraries(
  tune
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    sample
  RUNTIME
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    tune
  RUNTIME
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
