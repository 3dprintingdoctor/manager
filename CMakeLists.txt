cmake_minimum_required(VERSION 2.8.3)
project(manager)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs)

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

add_executable(manager_node src/manager_node.cpp)

target_link_libraries(manager_node ${catkin_LIBRARIES})
