cmake_minimum_required(VERSION 3.0.2)
project(prm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  a4_setup
  cv_bridge
  image_transport
  message_generation
  sensor_msgs
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system signals)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

find_package(OpenCV)
find_package(Eigen3 REQUIRED)


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
    #  INCLUDE_DIRS include
    #  LIBRARIES drawing_tools
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/main.cpp src/prminterface/prminterface.cpp src/prm/prm.cpp)

## Add cmake target dependencies of the executable
## This enforces generating messages before building these executables
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/build_test.cpp src/prm/prm.cpp)
if(TARGET ${PROJECT_NAME}-test)
   message ( STATUS " catkin_LIBRARIES ${catkin_LIBRARIES}")

   target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endif()

