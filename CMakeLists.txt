cmake_minimum_required(VERSION 2.6)
project(modelbuilder)

find_package(PCL 1.5 REQUIRED PATHS /opt/ros/fuerte/share NO_DEFAULT_PATH)
find_package(OpenCV 2.4 REQUIRED PATHS /opt/ros/fuerte/share NO_DEFAULT_PATH)
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")

add_subdirectory(src)
