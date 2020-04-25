# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

project (A2) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library(ranger_lib STATIC ranger.cpp rangerfusion.cpp laser.cpp sonar.cpp vector2.h)
set_target_properties(ranger_lib  PROPERTIES PUBLIC_HEADER "ranger.h;rangerfusion.h;laser.h;sonar.h;vector2.h")


add_library(cell cell.cpp)
set_target_properties(cell PROPERTIES PUBLIC_HEADER "cell.h")

#install
#set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}")
#install(TARGETS ranger_lib cell DESTINATION lib PUBLIC_HEADER DESTINATION include)


add_executable(assignment2 main.cpp)
target_link_libraries(assignment2 ranger_lib cell)
