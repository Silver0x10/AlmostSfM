
add_executable(sfm structure_from_motion.cpp
    ../src/utils.cpp
    ../src/init_translations.cpp
    ../src/relative_position_calculator.cpp
    ../src/triangulation.cpp
    ../src/bundle_adjustment.cpp
)
target_link_libraries(sfm ${OpenCV_LIBS})

# add_executable(test_BA test_bundle_adjustment.cpp
#     ../src/utils.cpp
#     ../src/bundle_adjustment.cpp
# )