cmake_minimum_required(VERSION 3.0.2)
project(motor_response)

add_compile_options(-std=c++11)

find_package(
  catkin
  REQUIRED COMPONENTS
    roscpp
    str1ker
    control_toolbox
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    actionlib
    control_toolbox
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${str1ker_INCLUDE_DIRS}
)

add_executable(
  sample
  sample.cpp
)

add_executable(
  tune
  tune.cpp
)


add_dependencies(
  ${catkin_EXPORTED_TARGETS}
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
