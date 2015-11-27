cmake_minimum_required(VERSION 2.8.3)
project(eecs600_rahul_pcl_utils)

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS cwru_pcl_utils tom_project_pcl kristina_hmi)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(cwru_pcl_utils)
find_package(baxter_cartesian_moves)
find_package(tom_project_pcl)
find_package( kristina_hmi)
find_package(marc_gripper )
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${cwru_pcl_utils_INCLUDE_DIR})
include_directories(${tom_project_pcl_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${kristina_hmi_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   
#cs_add_library(cwru_pcl_utils src/cwru_pcl_utils.cpp)  

# Executables
# cs_add_executable(example src/example.cpp)
# target_link_library(example my_lib)
cs_add_executable(eecs600Alpha src/baxter_cart_move_lib.cpp src/eecs_600_rahul.cpp)
target_link_libraries(eecs600Alpha blockfinder_lib cwru_pcl_utils cart_path_planner_lib ${catkin_LIBRARIES})

cs_install()
cs_export()
    
