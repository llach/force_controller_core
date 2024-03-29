cmake_minimum_required(VERSION 3.0.2)
project(force_controller_core)

# Default to C++14
if ("${CMAKE_VERSION}" VERSION_LESS "3.1")
    add_compile_options(-std=c++14)
else ()
    set(CMAKE_CXX_STANDARD 14)
endif ()

SET(ROS TRUE CACHE BOOL "ROS")

if (${ROS})
    message("----Force controller running with ROS----")
    find_package(catkin REQUIRED COMPONENTS
            roscpp
            )

    catkin_package(
            CATKIN_DEPENDS
            INCLUDE_DIRS include
            LIBRARIES ${PROJECT_NAME}
    )

    include_directories(${catkin_INCLUDE_DIRS})

    add_library(${PROJECT_NAME}
            include/force_controller_core/force_controller.h
            src/force_controller.cpp
            )

    target_include_directories(${PROJECT_NAME}
            PUBLIC include/
            PUBLIC ${catkin_INCLUDE_DIRS}
            )
    target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
    add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

    # Install
    install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

    install(TARGETS ${PROJECT_NAME}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
else ()
    message("----Force controller without ROS")

    include_directories(include/)

    add_library(${PROJECT_NAME}
            include/force_controller_core/force_controller.h
            src/force_controller.cpp
            )

    target_include_directories(${PROJECT_NAME} PUBLIC /force_controller_core/include)

    target_link_libraries(${PROJECT_NAME}  PUBLIC /force_controller_core/build/libforce_controller_core.a)

    install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        COMPONENT library
    )

    install(
        DIRECTORY include/force_controller_core
        DESTINATION include
    )

endif ()
