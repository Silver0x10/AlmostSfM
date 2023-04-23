
add_executable(sfm structure_from_motion.cpp
    ../src/utils.cpp
    ../src/init_translations.cpp
)

add_executable(BA test_bundle_adjustment.cpp
    ../src/utils.cpp
    ../src/bundle_adjustment.cpp
)