cmake_minimum_required(VERSION 3.0.2)
project(project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  dynamic_reconfigure
  message_generation
  tf
  tf2
  tf2_ros
)

add_message_files(
  FILES
  custom.msg
)

add_service_files(
  FILES
  paramsToGivenPose.srv
  calibration.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include  ${catkin_INCLUDE_DIRS}
)

add_executable(velocity_computer src/velComp.cpp)
target_link_libraries(velocity_computer ${catkin_LIBRARIES})
add_dependencies(velocity_computer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(odometry_computer src/odomComp.cpp)
target_link_libraries(odometry_computer ${catkin_LIBRARIES})
add_dependencies(odometry_computer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tf2_broad src/tfBroadcaster.cpp)
target_link_libraries(tf2_broad ${catkin_LIBRARIES})
add_dependencies(tf2_broad ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(rpm_computer src/rpmComp.cpp)
target_link_libraries(rpm_computer ${catkin_LIBRARIES})
add_dependencies(rpm_computer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})