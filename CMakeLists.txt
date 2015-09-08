cmake_minimum_required(VERSION 2.8.3)
project(voronoi_planner)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
    dynamicvoronoi
)

generate_dynamic_reconfigure_options(
  cfg/VoronoiPlanner.cfg
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -L libdynamic_voronoi")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} dynamic_voronoi
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
    dynamicvoronoi
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/planner_core.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

#add_executable(planner
#  src/example.cpp
#)
#target_link_libraries(planner
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

install(TARGETS ${PROJECT_NAME} #planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(FILES bgp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
