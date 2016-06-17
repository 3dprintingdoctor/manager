cmake_minimum_required(VERSION 2.8.3)
project(manager)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED NO_MODULE)
find_package(kdl_parser)


catkin_package(CATKIN_DEPENDS roscpp std_msgs manager_rec_msgs tf)

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS} ${kdl_parser_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include) #és provisional, per a que el QT autocompleti, pero corrent amb ROS, ja detecta els includes

### iknum ###
include_directories(/home/users/aleix.ripoll/iknum_ws/iknumlib)
link_directories(/home/users/aleix.ripoll/iknum_ws/build)

add_executable(manager_node src/manager_node.cpp)
target_link_libraries(manager_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} iknum)

add_dependencies(manager_node ${catkin_EXPORTED_TARGETS})
