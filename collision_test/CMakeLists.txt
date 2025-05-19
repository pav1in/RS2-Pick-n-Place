cmake_minimum_required(VERSION 3.0.2)
project(collision_test)

find_package(CGAL CONFIG REQUIRED)
find_package(CGAL COMPONENTS Qt5)
find_package(objectLib REQUIRED)


add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} 
                        objectLib::objectLib)




