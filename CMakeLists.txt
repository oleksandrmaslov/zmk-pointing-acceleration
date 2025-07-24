
zephyr_library()
zephyr_library_sources(src/input_processor_accel.c)
zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dts/bindings)
zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dts/)
zephyr_library_include_directories(${ZMK_BASE}/app/include)