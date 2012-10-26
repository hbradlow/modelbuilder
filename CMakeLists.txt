cmake_minimum_required(VERSION 2.6)
project(modelbuilder)

find_package(PCL 1.5 REQUIRED PATHS /opt/ros/fuerte/share NO_DEFAULT_PATH)
find_package(OpenCV 2.4 REQUIRED PATHS /opt/ros/fuerte/share NO_DEFAULT_PATH)

add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")

add_subdirectory(src)
