cmake_minimum_required(VERSION 2.8.3)
project(evaluate_utias_dataset)

find_package(cmake_modules REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs 
  tf 
  geometry_msgs     
  nav_msgs
  rosbag
  cv_bridge
  message_generation
  image_transport
  mhls_utias_dataset
)

find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})



catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(evaluate_utias_dataset src/evaluate_utias_dataset.cpp)
target_link_libraries(evaluate_utias_dataset ${catkin_LIBRARIES} ${rosbag_LIBRARIES})

add_executable(save_results_utias_dataset src/save_results_utias_dataset.cpp)
target_link_libraries(save_results_utias_dataset ${catkin_LIBRARIES})
