cmake_minimum_required(VERSION 3.0.2)
project(collision_avoid)

find_package(CGAL CONFIG REQUIRED)
find_package(CGAL COMPONENTS Qt5)



add_library(objectLib src/obstacle_manager.cpp)

add_definitions(-DCGAL_USE_BASIC_VIEWER)

target_link_libraries(objectLib
                        PUBLIC 
                        CGAL::CGAL
                        CGAL::CGAL_Qt5
                        CGAL::CGAL_Basic_viewer
                        )

# set_target_properties(object_lib
#                         PROPERTIES
#                         CMAKE_CXX_STANDARD )

include(GNUInstallDirs)


target_include_directories(objectLib
                            PUBLIC
                            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")       

install(TARGETS
    objectLib
    EXPORT objectLibTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES src/obstacle_manager.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT objectLibTargets
        FILE objectLibTargets.cmake
        NAMESPACE objectLib::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/objectLib
        )

include(CMakePackageConfigHelpers)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/objectLibConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/objectLib
)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/objectLibConfig.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/objectLib)



