include_directories(
    ${PCL_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIR}
    ${JSON_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${BULLETSIM_SOURCE_DIR}/src
    ${BULLET_DIR}/src
    ${WM_INCLUDE_DIR}
)

add_library(clouds get_chessboard_pose.cpp cloud_ops.cpp utils_pcl.cpp utils_cv.cpp utils_vector.cpp)
target_link_libraries(clouds ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(calculate_checkerboard_transforms calculate_checkerboard_transforms.cpp)
target_link_libraries(calculate_checkerboard_transforms clouds ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(concatenate_clouds concatenate_clouds.cpp)
target_link_libraries(concatenate_clouds clouds ${OpenCV_LIBS} ${PCL_LIBRARIES})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
