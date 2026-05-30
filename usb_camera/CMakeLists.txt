cmake_minimum_required(VERSION 3.0.2)
project(usb_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(img_publisher src/img_publisher.cpp)
target_link_libraries(img_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(img_viewer src/img_viewer.cpp)
target_link_libraries(img_viewer ${catkin_LIBRARIES} ${OpenCV_LIBS})
