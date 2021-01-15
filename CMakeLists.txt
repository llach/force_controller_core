cmake_minimum_required(VERSION 3.0.2)
project(force_controller_core)

# Default to C++14
if ("${CMAKE_VERSION}" VERSION_LESS "3.1")
    add_compile_options(-std=c++14)
else ()
    set(CMAKE_CXX_STANDARD 14)
endif ()


SET(YARP FALSE CACHE BOOL "YARP")

if (NOT ${YARP})
=======
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
    message("----Force controller running with YARP----")

    include_directories(include/)

    include(GNUInstallDirs)

    if(NOT CMAKE_CONFIGURATION_TYPES)
        if(NOT CMAKE_BUILD_TYPE)
            message(STATUS "Setting build type to 'Release' as none was specified.")
            set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
        endif()
    endif()

    find_package(YARP COMPONENTS os sig math dev REQUIRED)

=======
    message("----Force controller without ROS")

    include_directories(include/)

    add_library(${PROJECT_NAME}
            include/force_controller_core/force_controller.h
            src/force_controller.cpp
            )

    target_include_directories(${PROJECT_NAME} PUBLIC /force_controller_core/include)

    target_link_libraries(${PROJECT_NAME}
                            PUBLIC
                            /force_controller_core/build/libforce_controller_core.a
                            PRIVATE YARP::YARP_os
                                    YARP::YARP_init
                                    YARP::YARP_dev
                                    YARP::YARP_math
                                    YARP::YARP_sig)

                                # you can specify the CMAKE_INSTALL_BINDIR adding -DCMAKE_INSTALL_PREFIX=/path at the cmake .. command
                                install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})
=======
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
