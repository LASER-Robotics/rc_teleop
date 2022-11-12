cmake_minimum_required(VERSION 2.8.3)
project(rc_teleop)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  std_msgs
  visualization_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
    DEPENDS
)

add_executable(rc_teleop src/rc_teleop.cpp)
add_executable(rc_teleop_txs src/rc_teleop_txs.cpp)

target_link_libraries(rc_teleop ${catkin_LIBRARIES})
add_dependencies(rc_teleop std_msgs ${catkin_EXPORTED_TARGETS})

install(TARGETS rc_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

target_link_libraries(rc_teleop_txs ${catkin_LIBRARIES})
add_dependencies(rc_teleop_txs std_msgs ${catkin_EXPORTED_TARGETS})

install(TARGETS rc_teleop_txs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)