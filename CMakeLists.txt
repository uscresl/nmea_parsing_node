cmake_minimum_required(VERSION 2.8.3)
project(nmea_parsing_node)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Boost REQUIRED COMPONENTS thread system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_executable(nmea_parsing_node_test src/nmea_parsing_node_test.cpp)

target_link_libraries(nmea_parsing_node_test
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
