cmake_minimum_required(VERSION 2.8.3)
project(vision_bridge)
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  geometry_msgs
  message_generation
)

set(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath=/home/fshs/anaconda3/lib")

add_service_files(
  FILES
  detection.srv
)

add_message_files(
  FILES
  ObjectInfo.msg
  ObjectArray.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge
)

LINK_DIRECTORIES(/home/fshs/work/hirop_vision/out/libs)

add_definitions(-D__LINUX__)

include_directories(
  include
  /home/fshs/work/hirop_vision/out/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vision_bridge src/main.cpp src/ros_detector.cpp)
add_dependencies(vision_bridge  ${vision_bridge_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(vision_bridge
   hirop_vision
   ${catkin_LIBRARIES}
 )



