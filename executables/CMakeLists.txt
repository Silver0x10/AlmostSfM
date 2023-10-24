
add_executable(sfm structure_from_motion.cpp
    ../src/utils.cpp
    ../src/init_translations.cpp
    ../src/relative_position_calculator.cpp
    ../src/triangulation.cpp
    ../src/bundle_adjustment.cpp
    ../src/evaluation.cpp
    ../src/icp_3d.cpp
)
target_link_libraries(sfm ${OpenCV_LIBS})

# add_executable(test_BA test_bundle_adjustment.cpp
#     ../src/utils.cpp
#     ../src/bundle_adjustment.cpp
# )

add_executable(test_relative_pos test_relative_pos.cpp
    ../src/relative_position_calculator.cpp
    ../src/utils.cpp
    ../src/triangulation.cpp
)
target_link_libraries(test_relative_pos ${OpenCV_LIBS})

add_executable(test_triangulation test_triangulation.cpp
    ../src/utils.cpp
    ../src/triangulation.cpp
    ../src/icp_3d.cpp
)
target_link_libraries(test_triangulation ${OpenCV_LIBS})

add_executable(test_icp_3d test_icp_3d.cpp
    ../src/utils.cpp
    ../src/icp_3d.cpp
    ../src/evaluation.cpp
)
target_link_libraries(test_icp_3d ${OpenCV_LIBS})