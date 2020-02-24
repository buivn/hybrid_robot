cmake_minimum_required(VERSION 2.8.3)
project(pcd_filter)


add_compile_options(-std=c++11)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# find system dependencies
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
link_directories(${Eigen3_LIBRARY_DIRS})
add_definitions(${Eigen3_DEFINITIONS})

find_package(Boost REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system thread)

# find catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
  sensor_msgs
  pcl_conversions
  roscpp
  cmake_modules
  geometry_msgs
  eigen_conversions
  tf
  urdf
  moveit_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools

)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)


# generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(
INCLUDE_DIRS include
LIBRARIES target_plane
CATKIN_DEPENDS cmake_modules eigen_conversions geometry_msgs message_runtime roscpp sensor_msgs std_msgs moveit_msgs trajectory_msgs moveit_core moveit_ros_planning  moveit_ros_planning_interface 
DEPENDS Eigen PCL Boost
)
# do not wildcard install files since the root folder of the package will contain a debian folder for releasing

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# add_library(target_plane src/target_plane.cpp)
# add_dependencies(target_plane ${${PPROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# set(SOURCES src/pcd_filter.cpp)
# add_executable(pcd_filter ${SOURCES})
add_executable(pcd_filter src/pcd_filter.cpp)
# add_executable(crp src/climbing_robot_project.cpp)
add_executable(crp_bu src/climbing_robot_project_backup.cpp)
add_executable(target_pose src/target_pose.cpp)
add_executable(transform src/transform.cpp)
add_executable(process_trajectory src/process_trajectory.cpp)
add_executable(after_process_trajectory src/after_process_trajectory.cpp)

# set_target_properties(crp PROPERTIES OUTPUT_NAME climbing_robot_project PREFIX "")
# add_dependencies(crp ${${PPROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(crp_bu ${${PPROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pcd_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# target_link_libraries(crp target_plane ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(crp_bu ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(target_pose ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(transform ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(process_trajectory ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(after_process_trajectory ${catkin_LIBRARIES})
# target_link_libraries(target_plane ${catkin_LIBRARIES})


