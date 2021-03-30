# CMakeList.txt : CMake project for P4_project, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 2.6)

project ("P4_project")
find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



#find_package(realsense2)
#include_directories(${realsense2_INCLUDE_DIRS})
#link_directories(${realsense2_LIBRARY_DIRS})                                                                         4
#target_link_libraries(P4_project ${realsense2_LIBRARY})

set(REALSENSE_INSTALL_PATH CACHE PATH "C:/Program Files (x86)/Intel RealSense SDK 2.0")
include_directories(${REALSENSE_INSTALL_PATH}/include/)
link_directories(${REALSENSE_INSTALL_PATH}/lib/x64)
set(REALSENSE_LIBS "realsense2.lib")
link_libraries(${REALSENSE_LIBS})

add_executable (P4_project P4_project.cpp)

target_link_libraries (P4_project ${PCL_LIBRARIES})

# Add source to this project's executable.
#link_directories(${OPENNI2_LIB64}/OpenNI2.lib)


#include_directories( ${OpenCV_INCLUDE_DIRS} )
#target_link_libraries(P4_project ${OpenCV_LIBS} )



# TODO: Add tests and install targets if needed.

