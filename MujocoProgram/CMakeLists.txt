cmake_minimum_required(VERSION 3.8)

project("Mujoco_Sim")


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source to this project's executable.
set(MUJOCO_MJPRO_PATH "C:\\mujoco200_win64\\bin")
set(MUJOCO_MJPRO_INCLUDE "C:\\mujoco200_win64\\include")

#Options
set(SRCS main.cpp)
set(BIN_NAME mujoco_exe)

#Paths
include_directories(${MUJOCO_MJPRO_INCLUDE})
link_directories(${MUJOCO_MJPRO_PATH})


#set(REALSENSE_INSTALL_PATH CACHE PATH "C:/Program Files(x86)/IntelRealSenseSDK2.0")
#include_directories(${REALSENSE_INSTALL_PATH}/include/)
#link_directories(${REALSENSE_INSTALL_PATH}/lib/x64)
#set(REALSENSE_LIBS "realsense2.lib")
#link_libraries(${REALSENSE_LIBS})


set(PROJECT_SOURCES
        main.cpp
)
add_executable(Mujoco_Sim
 ${PROJECT_SOURCES}
 )


target_link_libraries(
Mujoco_Sim
"mujoco200.dll"
"glfw3.dll"
)
