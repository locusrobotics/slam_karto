cmake_minimum_required(VERSION 3.0.2)
project(slam_karto)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    message_filters
    message_generation
    nav_msgs
    open_karto
    rosconsole
    roscpp
    sensor_msgs
    sparse_bundle_adjustment
    std_msgs
    std_srvs
    tf
    tf2
    tf2_ros
    visualization_msgs
)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
    SetMapTransform.srv
)
generate_messages(
  DEPENDENCIES
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

add_executable(slam_karto
  src/loop_closure_callback.cpp
  src/slam_karto.cpp
  src/spa_solver.cpp
)
add_dependencies(slam_karto
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(slam_karto
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS slam_karto
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
