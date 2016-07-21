cmake_minimum_required(VERSION 2.8.3)
project(manager)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/Modules)

find_package(GA2H REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED NO_MODULE)
find_package(kdl_parser)
find_package(PCL 1.7 REQUIRED COMPONENTS common io)


catkin_package(CATKIN_DEPENDS roscpp std_msgs manager_rec_msgs tf)


INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS} ${kdl_parser_INCLUDE_DIRS} ${GA2H_DIRS} ${PCL_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include) #és provisional, per a que el QT autocompleti, pero corrent amb ROS, ja detecta els includes

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

### iknum ###
include_directories(/home/users/aleix.ripoll/iknum_ws/iknumlib)
link_directories(/home/users/aleix.ripoll/iknum_ws/build)

add_executable(manager_node src/manager_node.cpp)
target_link_libraries(manager_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} ${GA2H_LIBRARY} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} iknum)

add_dependencies(manager_node ${catkin_EXPORTED_TARGETS})
