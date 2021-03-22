# CMakeList.txt : CMake project for P4_project, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

project ("P4_project")
find_package(OpenCV REQUIRED)
find_package(realsense2)
find_package(PCL 1.11 REQUIRED)
add_executable (P4_project "P4_project.cpp" "P4_project.h")
target_link_libraries(P4_project ${realsense2_LIBRARY})
# Add source to this project's executable.

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(P4_project ${OpenCV_LIBS} )

# TODO: Add tests and install targets if needed.

