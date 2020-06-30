cmake_minimum_required(VERSION 3.0.2)
project(force_controller_core)

# Default to C++14
if ("${CMAKE_VERSION}" VERSION_LESS "3.1")
    add_compile_options(-std=c++14)
else ()
    set(CMAKE_CXX_STANDARD 14)
endif ()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        controller_interface
        joint_trajectory_controller
        )

catkin_package(
        CATKIN_DEPENDS
        joint_trajectory_controller
        INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include/)

# Install
install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
