cmake_minimum_required(VERSION 2.8.3)
project(slam_karto)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    geometry_msgs
    interactive_markers
    message_filters
    nav_msgs
    open_karto
    rosconsole
    roscpp
    sensor_msgs
    sparse_bundle_adjustment
    std_msgs
    tf
    tf2
    tf2_ros
    visualization_msgs
)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package()

add_executable(map_alignment_tool
  src/map_alignment_tool.cpp
)
target_link_libraries(map_alignment_tool
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(slam_karto
  src/loop_closure_callback.cpp
  src/slam_karto.cpp
  src/spa_solver.cpp
)
target_link_libraries(slam_karto
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS map_alignment_tool slam_karto
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
