cmake_minimum_required(VERSION 2.8.3)
project(cart_traj_generators)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
)

catkin_package(
  DEPENDS  orocos_kdl
  CATKIN_DEPENDS
  INCLUDE_DIRS include
  LIBRARIES cart_traj_generator circle_traj_generator line_traj_generator line_periodic_traj_generator
)

add_library(cart_traj_generator src/CartTrajGenerator.cpp)
target_link_libraries(cart_traj_generator ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(circle_traj_generator src/CircleTrajGenerator.cpp)
target_link_libraries(circle_traj_generator ${catkin_LIBRARIES} cart_traj_generator ${orocos_kdl_LIBRARIES})

add_library(line_traj_generator src/LineTrajGenerator.cpp)
target_link_libraries(line_traj_generator ${catkin_LIBRARIES} cart_traj_generator ${orocos_kdl_LIBRARIES})

add_library(line_periodic_traj_generator src/LinePeriodicTrajGenerator.cpp)
target_link_libraries(line_periodic_traj_generator ${catkin_LIBRARIES} cart_traj_generator line_traj_generator ${orocos_kdl_LIBRARIES})

install(TARGETS cart_traj_generator circle_traj_generator line_traj_generator line_periodic_traj_generator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

