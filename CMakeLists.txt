cmake_minimum_required(VERSION 3.0.2)
project(test_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  tf2_ros
  tf2
  message_generation
  dynamic_reconfigure
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
 cfg/DeltaExt.cfg
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_rs
#  CATKIN_DEPENDS roscpp rospy sensor_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
)



## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/test_tools.cpp)
add_dependencies(${PROJECT_NAME}_node 
${catkin_EXPORTED_TARGETS}
${PROJECT_NAME}_gencfg)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(set_default_param src/set_default_param.cpp)
add_dependencies(set_default_param 
${catkin_EXPORTED_TARGETS}
${PROJECT_NAME}_gencfg)
## Specify libraries to link a library or executable target against
target_link_libraries(set_default_param
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)