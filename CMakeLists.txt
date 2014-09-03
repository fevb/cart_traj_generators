cmake_minimum_required(VERSION 2.8.3)
project(cart_traj_generators)

find_package(catkin REQUIRED COMPONENTS orocos_kdl)

catkin_package(
  DEPENDS 
  CATKIN_DEPENDS orocos_kdl
  INCLUDE_DIRS include
  LIBRARIES cart_traj_generator circle_traj_generator line_traj_generator line_periodic_traj_generator
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(cart_traj_generator src/CartTrajGenerator.cpp)
target_link_libraries(cart_traj_generator ${catkin_LIBRARIES})

add_library(circle_traj_generator src/CircleTrajGenerator.cpp)
target_link_libraries(circle_traj_generator ${catkin_LIBRARIES} cart_traj_generator)

add_library(line_traj_generator src/LineTrajGenerator.cpp)
target_link_libraries(line_traj_generator ${catkin_LIBRARIES} cart_traj_generator)

add_library(line_periodic_traj_generator src/LinePeriodicTrajGenerator.cpp)
target_link_libraries(line_periodic_traj_generator ${catkin_LIBRARIES} cart_traj_generator line_traj_generator)

install(TARGETS cart_traj_generator circle_traj_generator line_traj_generator line_periodic_traj_generator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

