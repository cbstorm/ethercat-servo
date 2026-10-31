set(ETHERCAT_MOTION_EXAMPLES basic_axis multi_axis mqtt_control simulation)

foreach(example ${ETHERCAT_MOTION_EXAMPLES})
    add_executable(${example} ${example}.cpp example_common.hpp)
    target_link_libraries(${example} PRIVATE ethercat_motion)
    target_compile_definitions(${example} PRIVATE ETHERCAT_MOTION_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config")
    target_compile_options(${example} PRIVATE -Wall -Wextra)
endforeach()

# Plain C program using the C API.
add_executable(c_api_example c_api_example.c)
target_link_libraries(c_api_example PRIVATE ethercat_motion)
target_compile_definitions(c_api_example PRIVATE ETHERCAT_MOTION_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config")
set_target_properties(c_api_example PROPERTIES C_STANDARD 99 LINKER_LANGUAGE CXX)
