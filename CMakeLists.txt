cmake_minimum_required(VERSION 3.8)
project(uav_target_tracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED) # 如果采用路线A

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 编译气球检测节点
add_executable(detect_balloon src/detect_balloon.cpp)
ament_target_dependencies(detect_balloon
  rclcpp sensor_msgs geometry_msgs cv_bridge image_transport tf2 tf2_ros tf2_geometry_msgs px4_msgs
)
target_link_libraries(detect_balloon ${OpenCV_LIBS})

# 编译门框检测节点
add_executable(detect_doorframe src/detect_doorframe.cpp)
ament_target_dependencies(detect_doorframe
  rclcpp sensor_msgs geometry_msgs cv_bridge image_transport tf2 tf2_ros tf2_geometry_msgs px4_msgs
)
target_link_libraries(detect_doorframe ${OpenCV_LIBS})

install(DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)
ament_package()