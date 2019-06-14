cmake_minimum_required(VERSION 2.8.3)
project(vision_bridge)
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  geometry_msgs
  message_generation
)


add_service_files(
  FILES
  detection.srv
  listDetector.srv
  listObject.srv
  train.srv
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

LINK_DIRECTORIES(/home/eima/Work/hirop/build/install/lib)

add_definitions(-D__LINUX__)

include_directories(
  include
  /home/eima/Work/hirop/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vision_bridge src/main.cpp src/ros_detector.cpp src/ros_trainer.cpp)

## 当工程以来与工程内自身的消息或服务时，需添加以下内容
add_dependencies(vision_bridge  ${vision_bridge_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(vision_bridge
   hirop_vision
   ${catkin_LIBRARIES}
 )



