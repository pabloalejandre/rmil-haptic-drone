cmake_minimum_required(VERSION 2.8.3)
project(rmil_drone_practicals)
add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  tf
  cv_bridge
  image_transport
  roscpp
  nav_msgs
  sensor_msgs
  std_msgs
  interactive_markers

  rospy
  eigen_conversions
  geometry_msgs
  #tf_conversions
  message_generation
  pluginlib
  realtime_tools
)

#find_path(DHD_INCLUDE_DIR NAMES "dhdc.h" PATHS "/home/student/sdk-3.14.0/include")
find_path(DHD_INCLUDE_DIR NAMES "dhdc.h" PATHS "/home/pablo/sdk-3.14.0/include")
message("+++++++++++++++++++++${DHD_INCLUDE_DIR}+++++++++++++++++++++++++++++")
if(NOT DHD_INCLUDE_DIR)
  message("LIBRARY FOR DHD was not found. If you do not have it you will not be able to use the Omega Haptic device.") 
  return()
endif(NOT DHD_INCLUDE_DIR) 
include_directories(include ${catkin_INCLUDE_DIRS} ${DHD_INCLUDE_DIR}) 


#find_library(DHDLIB NAMES "libdhd.a"  PATHS "/home/student/sdk-3.14.0/lib/release/lin-x86_64-gcc")
find_library(DHDLIB NAMES "libdhd.a"  PATHS "/home/pablo/sdk-3.14.0/lib/release/lin-x86_64-gcc")
message("+++++++++++++++++++++++++++++++++++${DHDLIB}++++++++++++++++++++++++++++++++++++++++")
link_directories(${DHDLIB})


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
include_directories( SYSTEM ${EIGEN3_INCLUDE_DIRS} )
#find_package( Curses REQUIRED )
INCLUDE(FindPkgConfig)
PKG_SEARCH_MODULE(SDL2 REQUIRED sdl2)
#PKG_SEARCH_MODULE(SDL2_image REQUIRED sdl2_image)

## BRISK 2 import
include(ExternalProject)
include_directories(${CMAKE_BINARY_DIR}/include)
link_directories(${CMAKE_BINARY_DIR}/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mssse3")
ExternalProject_Add(brisk_external
  URL "https://www.doc.ic.ac.uk/~sleutene/software/brisk-cameraaware.zip"
  INSTALL_DIR ${CMAKE_BINARY_DIR}
  CMAKE_ARGS -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/brisk
  BUILD_IN_SOURCE 0
  BUILD_COMMAND make -j${N_CORES}
  INSTALL_COMMAND make install
)
add_library(brisk STATIC IMPORTED)
add_library(agast STATIC IMPORTED)
set(BRISK_LIBRARIES brisk agast)
set_target_properties(brisk PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/libbrisk.a)
set_target_properties(agast PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/libagast.a)
add_dependencies(brisk brisk_external)
add_dependencies(agast brisk_external)
# install it once built
install(FILES ${CMAKE_BINARY_DIR}/lib/libbrisk.a ${CMAKE_BINARY_DIR}/lib/libagast.a
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
 install(DIRECTORY ${CMAKE_BINARY_DIR}/include/brisk/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/brisk/)
install(DIRECTORY ${CMAKE_BINARY_DIR}/lib/CMake/brisk/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/CMake/brisk/)
    
## DBoW2 import
ExternalProject_Add(DBoW2_external
    GIT_REPOSITORY https://github.com/dorian3d/DBoW2.git
    UPDATE_COMMAND ""
    INSTALL_DIR ${CMAKE_BINARY_DIR}
    CMAKE_ARGS
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
      -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/DBoW2
    BUILD_IN_SOURCE 0
    BUILD_COMMAND make -j${N_CORES}
    INSTALL_COMMAND make install
  )
add_library(DBoW2 SHARED IMPORTED)
add_dependencies(DBoW2 DBoW2_external)
set(DBoW2_LIBRARIES DBoW2)
if(APPLE)
  set_target_properties(DBoW2 PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/libDBoW2.dylib)
  # install it once built
  install(FILES ${CMAKE_BINARY_DIR}/DBoW2/lib/libDBoW2.dylib
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
  install(DIRECTORY ${CMAKE_BINARY_DIR}/include/DBoW2/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/DBoW2/)
else()
 set_target_properties(DBoW2 PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/libDBoW2.so)
 # install it once built
 install(FILES ${CMAKE_BINARY_DIR}/DBoW2/lib/libDBoW2.so
   DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
 install(DIRECTORY ${CMAKE_BINARY_DIR}/include/DBoW2/
   DESTINATION ${CMAKE_INSTALL_PREFIX}/include/DBoW2/)
endif()

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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

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
  INCLUDE_DIRS include
  LIBRARIES ardrone_practicals
  CATKIN_DEPENDS image_transport opencv2 roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/CameraBase.cpp
   src/Frontend.cpp
   src/Imu.cpp
   src/ViEkf.cpp
   src/FBrisk.cpp
   src/PidController.cpp
)
add_dependencies(${PROJECT_NAME} brisk agast)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${BRISK_LIBRARIES}
  ${DBoW2_LIBRARIES}
)

add_library(${PROJECT_NAME}_autopilot
   src/Autopilot.cpp
   src/VisualInertialTracker.cpp
)
target_link_libraries(${PROJECT_NAME}_autopilot
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_autopilot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(rmil_drone_node src/rmil_drone_node.cpp)

add_executable(leader_node src/leader.cpp)
target_include_directories(leader_node PUBLIC ${DHD_INCLUDE_DIR}) 
target_link_libraries(leader_node ${DHDLIB} usb-1.0 rt ncurses ${catkin_LIBRARIES}) #

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(rmil_drone_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(rmil_drone_node
  ${PROJECT_NAME}
  ${PROJECT_NAME}_autopilot
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${SDL2_LIBRARIES}
  ${BRISK_LIBRARIES}
  ${DBoW2_LIBRARIES}
)

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
# install(TARGETS ardrone_practicals ardrone_practicals_node
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

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  #catkin_add_gtest(${PROJECT_NAME}_test 
  #  test/test_rmil_drone_practicals_1.cpp
  #)
  #target_link_libraries(${PROJECT_NAME}_test 
  #  ${catkin_LIBRARIES}
  #  ${OpenCV_LIBRARIES}
  #  ${PROJECT_NAME}
  #  pthread
  #)
endif()
