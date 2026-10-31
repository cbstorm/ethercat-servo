cmake_minimum_required(VERSION 3.16)

project(ethercat_motion
    VERSION 1.0.0
    DESCRIPTION "Linux EtherCAT motion control library (IgH EtherCAT Master, CiA 402)"
    LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
set(ETHERCAT_MOTION_WITH_IGH "AUTO" CACHE STRING "IgH EtherCAT Master backend: AUTO, ON or OFF")
set_property(CACHE ETHERCAT_MOTION_WITH_IGH PROPERTY STRINGS AUTO ON OFF)
set(ETHERCAT_MOTION_WITH_MQTT "AUTO" CACHE STRING "MQTT adapter (Eclipse Paho MQTT C++): AUTO, ON or OFF")
set_property(CACHE ETHERCAT_MOTION_WITH_MQTT PROPERTY STRINGS AUTO ON OFF)
option(ETHERCAT_MOTION_BUILD_STATIC "Also build the static library" ON)
option(ETHERCAT_MOTION_BUILD_TESTS "Build unit tests (GoogleTest)" ON)
option(ETHERCAT_MOTION_BUILD_EXAMPLES "Build examples" ON)
option(ETHERCAT_MOTION_BUILD_TOOLS "Build command line tools" ON)
option(ETHERCAT_MOTION_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json 3.2 REQUIRED)

set(ETHERCAT_MOTION_HAVE_IGH OFF)
if(NOT ETHERCAT_MOTION_WITH_IGH STREQUAL "OFF")
    find_package(EtherCAT)
    if(EtherCAT_FOUND)
        set(ETHERCAT_MOTION_HAVE_IGH ON)
    elseif(ETHERCAT_MOTION_WITH_IGH STREQUAL "ON")
        message(FATAL_ERROR "ETHERCAT_MOTION_WITH_IGH=ON but IgH EtherCAT Master (ecrt.h / libethercat) was not "
                            "found. Set EtherCAT_ROOT (e.g. -DEtherCAT_ROOT=/opt/etherlab).")
    endif()
endif()

set(ETHERCAT_MOTION_HAVE_MQTT OFF)
if(NOT ETHERCAT_MOTION_WITH_MQTT STREQUAL "OFF")
    # PahoMqttCppConfig.cmake hard-requires these; probe them first so a
    # partial installation disables MQTT instead of failing the configure.
    find_package(eclipse-paho-mqtt-c QUIET CONFIG)
    find_package(OpenSSL QUIET)
    if(eclipse-paho-mqtt-c_FOUND AND OpenSSL_FOUND)
        find_package(PahoMqttCpp QUIET CONFIG)
    elseif(NOT ETHERCAT_MOTION_WITH_MQTT STREQUAL "ON")
        message(STATUS "ethercat_motion: Paho MQTT C (libpaho-mqtt-dev) or OpenSSL (libssl-dev) development files "
                       "missing - MQTT adapter disabled")
    endif()
    if(PahoMqttCpp_FOUND AND TARGET PahoMqttCpp::paho-mqttpp3)
        set(ETHERCAT_MOTION_HAVE_MQTT ON)
    elseif(ETHERCAT_MOTION_WITH_MQTT STREQUAL "ON")
        message(FATAL_ERROR "ETHERCAT_MOTION_WITH_MQTT=ON but Paho MQTT C++ was not found "
                            "(apt install libpaho-mqttpp-dev libpaho-mqtt-dev libssl-dev).")
    endif()
endif()

message(STATUS "ethercat_motion: IgH EtherCAT backend : ${ETHERCAT_MOTION_HAVE_IGH}")
message(STATUS "ethercat_motion: MQTT adapter        : ${ETHERCAT_MOTION_HAVE_MQTT}")
if(NOT ETHERCAT_MOTION_HAVE_IGH)
    message(STATUS "ethercat_motion: building WITHOUT hardware backend - only the simulation backend is usable")
endif()

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
configure_file(cmake/version.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/include/ethercat_motion/version.hpp" @ONLY)

set(ETHERCAT_MOTION_SOURCES
    src/common/types.cpp
    src/common/diagnostics.cpp
    src/common/rt.cpp
    src/ethercat/pdo_mapping.cpp
    src/ethercat/ethercat_slave.cpp
    src/ethercat/ethercat_master.cpp
    src/ethercat/igh_backend.cpp
    src/ethercat/simulation_backend.cpp
    src/cia402/cia402_drive.cpp
    src/control/pid_controller.cpp
    src/motion/trajectory.cpp
    src/motion/axis.cpp
    src/motion/axis_runtime.cpp
    src/motion/motion_controller.cpp
    src/motion/coordinator.cpp
    src/safety/safety_manager.cpp
    src/config/xml_reader.cpp
    src/config/esi_parser.cpp
    src/config/config.cpp
    src/mqtt/mqtt_codec.cpp
    src/mqtt/mqtt_adapter.cpp
    src/controller.cpp
    src/c_api.cpp
)

set(ETHERCAT_MOTION_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wcast-align -Woverloaded-virtual
                             -Wnull-dereference -Wdouble-promotion -Wformat=2 -Wimplicit-fallthrough)
if(ETHERCAT_MOTION_WARNINGS_AS_ERRORS)
    list(APPEND ETHERCAT_MOTION_WARNINGS -Werror)
endif()

# Compile once, link into both the shared and the static library.
add_library(ethercat_motion_objects OBJECT ${ETHERCAT_MOTION_SOURCES})
set_target_properties(ethercat_motion_objects PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_VISIBILITY_PRESET default)
target_include_directories(ethercat_motion_objects
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_compile_options(ethercat_motion_objects PRIVATE ${ETHERCAT_MOTION_WARNINGS})
target_link_libraries(ethercat_motion_objects
    PUBLIC Threads::Threads
    PRIVATE spdlog::spdlog yaml-cpp::yaml-cpp nlohmann_json::nlohmann_json)
if(ETHERCAT_MOTION_HAVE_IGH)
    target_compile_definitions(ethercat_motion_objects PRIVATE ETHERCAT_MOTION_HAVE_IGH=1)
    target_link_libraries(ethercat_motion_objects PRIVATE EtherCAT::EtherCAT)
endif()
if(ETHERCAT_MOTION_HAVE_MQTT)
    target_compile_definitions(ethercat_motion_objects PRIVATE ETHERCAT_MOTION_WITH_MQTT=1)
    target_link_libraries(ethercat_motion_objects PRIVATE PahoMqttCpp::paho-mqttpp3)
endif()

set(ETHERCAT_MOTION_PRIVATE_LIBS spdlog::spdlog yaml-cpp::yaml-cpp)
if(ETHERCAT_MOTION_HAVE_IGH)
    list(APPEND ETHERCAT_MOTION_PRIVATE_LIBS EtherCAT::EtherCAT)
endif()
if(ETHERCAT_MOTION_HAVE_MQTT)
    list(APPEND ETHERCAT_MOTION_PRIVATE_LIBS PahoMqttCpp::paho-mqttpp3)
endif()

function(ethercat_motion_setup_library target)
    target_include_directories(${target} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
    target_link_libraries(${target}
        PUBLIC Threads::Threads
        PRIVATE ${ETHERCAT_MOTION_PRIVATE_LIBS})
    target_compile_features(${target} PUBLIC cxx_std_17)
endfunction()

add_library(ethercat_motion SHARED $<TARGET_OBJECTS:ethercat_motion_objects>)
ethercat_motion_setup_library(ethercat_motion)
set_target_properties(ethercat_motion PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    EXPORT_NAME ethercat_motion)
add_library(ethercat_motion::ethercat_motion ALIAS ethercat_motion)

set(ETHERCAT_MOTION_INSTALL_TARGETS ethercat_motion)
if(ETHERCAT_MOTION_BUILD_STATIC)
    add_library(ethercat_motion_static STATIC $<TARGET_OBJECTS:ethercat_motion_objects>)
    ethercat_motion_setup_library(ethercat_motion_static)
    set_target_properties(ethercat_motion_static PROPERTIES OUTPUT_NAME ethercat_motion EXPORT_NAME ethercat_motion_static)
    add_library(ethercat_motion::ethercat_motion_static ALIAS ethercat_motion_static)
    list(APPEND ETHERCAT_MOTION_INSTALL_TARGETS ethercat_motion_static)
endif()

# ---------------------------------------------------------------------------
# Tests, examples, tools
# ---------------------------------------------------------------------------
if(ETHERCAT_MOTION_BUILD_TESTS)
    find_package(GTest)
    if(GTest_FOUND)
        enable_testing()
        add_subdirectory(tests)
    else()
        message(WARNING "GoogleTest not found - unit tests disabled (apt install libgtest-dev)")
    endif()
endif()
if(ETHERCAT_MOTION_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
if(ETHERCAT_MOTION_BUILD_TOOLS)
    add_subdirectory(tools)
endif()

# ---------------------------------------------------------------------------
# Installation and package export
# ---------------------------------------------------------------------------
install(TARGETS ${ETHERCAT_MOTION_INSTALL_TARGETS}
    EXPORT ethercat_motionTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY include/ethercat_motion DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/include/ethercat_motion/version.hpp"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ethercat_motion)
install(FILES config/example.yaml config/simulation.yaml
        DESTINATION ${CMAKE_INSTALL_DATADIR}/ethercat_motion/config)

set(ETHERCAT_MOTION_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/ethercat_motion)
install(EXPORT ethercat_motionTargets
    NAMESPACE ethercat_motion::
    DESTINATION ${ETHERCAT_MOTION_CMAKE_DIR})
configure_package_config_file(cmake/ethercat_motionConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/ethercat_motionConfig.cmake"
    INSTALL_DESTINATION ${ETHERCAT_MOTION_CMAKE_DIR})
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/ethercat_motionConfigVersion.cmake"
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/ethercat_motionConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/ethercat_motionConfigVersion.cmake"
    cmake/FindEtherCAT.cmake
    DESTINATION ${ETHERCAT_MOTION_CMAKE_DIR})

configure_file(cmake/ethercat_motion.pc.in "${CMAKE_CURRENT_BINARY_DIR}/ethercat_motion.pc" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ethercat_motion.pc" DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
