cmake_minimum_required(VERSION 2.8.3)
project(autobot)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(ZED 2.0 REQUIRED)
find_package(jetson-inference REQUIRED)
#find_package(Qt5Gui REQUIRED)
#find_package(Qt5Widgets REQUIRED)
#find_package(Qt5Core REQUIRED)
find_package(Qt4 REQUIRED)

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

SET(OCV_VERSION "3.2")
SET(CUDA_VERSION "8.0")
SET(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package(OpenCV ${OCV_VERSION} COMPONENTS core highgui imgproc REQUIRED)
find_package(CUDA ${CUDA_VERSION} REQUIRED)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  geometry_msgs
  #tf
  rosconsole 
  #opencv 
  image_transport 
  dynamic_reconfigure 
  tf2_ros
  nodelet
  cv_bridge
)

#  amcl
#  navigation
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  velocity_param.msg
  angle_param.msg
  drive_values.msg
  drive_param.msg
  pid_input.msg
  wall_dist.msg
  pathFinderState.msg
  bounding_box.msg
  detected_img.msg
  detected_object.msg
  compound_img.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AdjustWallDist.srv
  TogglePathFinder.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

generate_dynamic_reconfigure_options(
  cfg/autobot.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES autobot
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs geometry_msgs rosconsole image_transport dynamic_reconfigure tf2_ros cv_bridge
  #DEPENDS system_lib 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/autobot.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###############################################################################
# INCLUDES

# Specify locations of header files.
include_directories(
	${catkin_INCLUDE_DIRS}
	${CUDA_INCLUDE_DIRS}
	${ZED_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

###############################################################################


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/autobot_node.cpp)

###############################################################################
# EXECUTABLE

add_definitions(-std=c++11)
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") ## if using from pcl package (pcl/vtk bug)
set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} ${CUDA_npps_LIBRARY}
  ${OpenCV_LIBS})

add_library(${PROJECT_NAME} src/zed_wrapper_nodelet.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(${PROJECT_NAME}_zed src/zed_wrapper_node.cpp)
target_link_libraries(${PROJECT_NAME}_zed ${PROJECT_NAME} ${LINK_LIBRARIES})
add_dependencies(${PROJECT_NAME}_zed ${PROJECT_NAME}_gencfg)

#file(GLOB_RECURSE odj_detector_sources /media/ubuntu/SSD/jetson-inference/*.cu /media/ubuntu/SSD/jetson-inference/*.cpp /media/ubuntu/SSD/jetson-inference/*.h)

#cuda_add_library(obj_detector_cuda ${odj_detector_sources})

cuda_add_executable(obj_detector src/obj_detector.cpp)
target_link_libraries(obj_detector nvcaffe_parser nvinfer jetson-inference ${LINK_LIBRARIES} ${QT_LIBRARIES})

cuda_add_executable(obj_classifier src/obj_classifier.cpp)
target_link_libraries(obj_classifier nvcaffe_parser nvinfer jetson-inference ${LINK_LIBRARIES} ${QT_LIBRARIES})

cuda_add_executable(vision_detect src/vision_detect.cpp)
target_link_libraries(vision_detect nvcaffe_parser nvinfer jetson-inference ${LINK_LIBRARIES} ${QT_LIBRARIES})

cuda_add_executable(vision_classify src/vision_classify.cpp)
target_link_libraries(vision_classify nvcaffe_parser nvinfer jetson-inference ${LINK_LIBRARIES} ${QT_LIBRARIES})

###############################################################################

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
# FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
# add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

install(TARGETS
  autobot
  autobot_zed
#  obj_detector
#  obj_detector_cuda
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_autobot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
