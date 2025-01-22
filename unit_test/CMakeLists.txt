MESSAGE("Building Unit")

ADD_EXECUTABLE(test_catmull_rom_spline ${PROJECT_SOURCE_DIR}/unit_test/test_catmull_rom_spline.cc)
TARGET_LINK_LIBRARIES(test_catmull_rom_spline ${PROJECT_NAME})

ADD_EXECUTABLE(test_catmull_rom_spline_list ${PROJECT_SOURCE_DIR}/unit_test/test_catmull_rom_spline_list.cc)
TARGET_LINK_LIBRARIES(test_catmull_rom_spline_list ${PROJECT_NAME})
