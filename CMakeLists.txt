cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_NAME ur_modern_driver)
set(PROJECT_DESCRIPTION "ROS-free version of the deprecated ur_modern_driver")

project(${PROJECT_NAME} LANGUAGES CXX VERSION 1.0.0)

add_subdirectory(cmake)
add_subdirectory(src)
