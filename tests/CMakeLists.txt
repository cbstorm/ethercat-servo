include(GoogleTest)

set(ETHERCAT_MOTION_TESTS
    test_pid
    test_cia402
    test_trajectory
    test_motion
    test_config
    test_esi
    test_mqtt_codec
    test_queues
    test_safety
    test_c_api
)

foreach(test ${ETHERCAT_MOTION_TESTS})
    add_executable(${test} ${test}.cpp test_helpers.hpp)
    target_link_libraries(${test} PRIVATE ethercat_motion GTest::gtest_main nlohmann_json::nlohmann_json)
    target_include_directories(${test} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    target_compile_definitions(${test} PRIVATE ETHERCAT_MOTION_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
    target_compile_options(${test} PRIVATE -Wall -Wextra)
    gtest_discover_tests(${test} PROPERTIES TIMEOUT 120 DISCOVERY_TIMEOUT 30)
endforeach()
