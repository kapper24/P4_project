cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_STANDARD 11)

project ("mujoco")
# Add source to this project's executable.
set(MUJOCO_MJPRO_PATH "C://Users/kaspe/OneDrive/Desktop/p4lecture/mujoco200_win64/bin")
set(MUJOCO_MJPRO_INCLUDE "C://Users/kaspe/OneDrive/Desktop/p4lecture/mujoco200_win64/include")

#Options
set(SRCS mujoco.cpp)
set(BIN_NAME mujoco_exe)

#Paths
include_directories(${MUJOCO_MJPRO_INCLUDE})
link_directories(${MUJOCO_MJPRO_PATH})

#Finding main mujoco library

#Showing mujoco library found
add_executable (mujoco "mujoco.cpp")
target_link_libraries(mujoco "mujoco200.dll")
#target_link_libraries(mujoco libmujoco200nogl.dll)
#Standard libraries for GL
#target_link_libraries(${PROJECT_NAME}_exe GL GLU glut )

#Additional libraries from mujoco package
#target_link_libraries(${PROJECT_NAME}_exe libglew.dll )
#target_link_libraries(${PROJECT_NAME}_exe libglfw.dll )
#target_link_libraries(${PROJECT_NAME}_exe libglewegl.dll )
target_link_libraries(mujoco "glfw3.dll")
