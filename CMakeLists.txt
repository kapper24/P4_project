cmake_minimum_required(VERSION 3.8)

project("P4_project")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)
find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(REALSENSE_INSTALL_PATH CACHE PATH "C:/Program Files(x86)/IntelRealSenseSDK2.0")
include_directories(${REALSENSE_INSTALL_PATH}/include/)
link_directories(${REALSENSE_INSTALL_PATH}/lib/x64)
set(REALSENSE_LIBS "realsense2.lib")
link_libraries(${REALSENSE_LIBS})


set(PROJECT_SOURCES
        P4_project.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(P4_project
        ${PROJECT_SOURCES}
    )
else()
    if(ANDROID)
        add_library(P4_project SHARED
            ${PROJECT_SOURCES}
        )
    else()
        add_executable(P4_project
            ${PROJECT_SOURCES}
        )
    endif()
endif()


target_link_libraries(
P4_project
PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
${PCL_LIBRARIES}
)
