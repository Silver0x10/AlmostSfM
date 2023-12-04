
add_executable(test_data_loading test_data_loading.cpp
    ../src/utils.cpp
)

add_executable(test_sicp_3d test_sicp_3d.cpp
    ../src/utils.cpp
    ../src/sicp_3d.cpp
    ../src/evaluation.cpp
)
target_link_libraries(test_sicp_3d ${OpenCV_LIBS})

add_executable(test_triangulation test_triangulation.cpp
    ../src/utils.cpp
    ../src/triangulation.cpp
)
target_link_libraries(test_triangulation ${OpenCV_LIBS})

add_executable(test_relative_pos test_relative_pos.cpp
    ../src/relative_position_calculator.cpp
    ../src/utils.cpp
    ../src/triangulation.cpp
)
target_link_libraries(test_relative_pos ${OpenCV_LIBS})

add_executable(test_init_t test_init_translations.cpp
    ../src/relative_position_calculator.cpp
    ../src/utils.cpp
    ../src/sicp_3d.cpp
    ../src/triangulation.cpp
    ../src/init_translations.cpp
)
target_link_libraries(test_init_t ${OpenCV_LIBS})

add_executable(test_BA test_bundle_adjustment.cpp
    ../src/utils.cpp
    ../src/bundle_adjustment.cpp
    ../src/triangulation.cpp
    ../src/sicp_3d.cpp
)
target_link_libraries(test_BA ${OpenCV_LIBS})

add_executable(sfm structure_from_motion.cpp
    ../src/utils.cpp
    ../src/init_translations.cpp
    ../src/relative_position_calculator.cpp
    ../src/triangulation.cpp
    ../src/bundle_adjustment.cpp
    ../src/evaluation.cpp
    ../src/sicp_3d.cpp
)
target_link_libraries(sfm ${OpenCV_LIBS})