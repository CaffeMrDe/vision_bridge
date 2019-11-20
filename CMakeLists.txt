cmake_minimum_required(VERSION 2.8.3)
project(vision_bridge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  geometry_msgs
  message_generation
  tf
  image_transport
)

find_package(hirop_vision REQUIRED)
find_package(PCL REQUIRED)
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
  CATKIN_DEPENDS roscpp cv_bridge tf
)

add_definitions(-D__LINUX__)

include_directories(
  include
  ${hirop_vision_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(vision_bridge src/main.cpp src/ros_detector.cpp src/ros_trainer.cpp)

## 当工程以来与工程内自身的消息或服务时，需添加以下内容
add_dependencies(vision_bridge  ${vision_bridge_EXPORTED_TARGETS})

target_link_libraries(vision_bridge
   ${hirop_vision_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )
