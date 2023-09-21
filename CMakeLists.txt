cmake_minimum_required(VERSION 3.0.2)
project(motor_response)

add_compile_options(-std=c++11)

find_package(
  catkin
  REQUIRED COMPONENTS
    roscpp
    str1ker
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    actionlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${str1ker_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}
  motor_response.cpp
)

add_dependencies(
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME}
  RUNTIME
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
