# Добавляем исходный файл твоего нового input-процессора
zephyr_library_sources(src/input_processor_accel.c)

# Указываем, где находятся devicetree bindings (YAML-файлы)
zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dts/bindings)
