cmake_minimum_required(VERSION 2.8.3)
project(kristina_hmi)

find_package(catkin_simple REQUIRED)
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

# Executables
cs_add_executable(hand_detect_server src/hand_detect_server.cpp)
cs_add_executable(hand_detect_client src/hand_detect_client.cpp)
cs_add_executable(image_listener src/image_listener.cpp)
cs_add_executable(image_publisher src/image_publisher.cpp)

cs_install()
cs_export()
    
