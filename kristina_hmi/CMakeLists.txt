cmake_minimum_required(VERSION 2.8.3)
project(kristina_hmi)

find_package(catkin_simple REQUIRED)
find_package(catkin REQUIRED COMPONENTS cwru_pcl_utils)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()
include_directories(${cwru_pcl_utils_INCLUDE_DIR})
# Executables
#cs_add_executable(hand_detect_server src/hand_detect_server.cpp)
#cs_add_executable(hand_detect_client src/hand_detect_client.cpp)
cs_add_executable(hand_detect_publisher src/hand_detect_publisher.cpp)

target_link_libraries(hand_detect_publisher cwru_pcl_utils ${catkin_LIBRARIES})

cs_install()
cs_export()
    
