cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME aitrackinghmd)
set(DRIVER_NAME "driver_${TARGET_NAME}")

project(${TARGET_NAME})

set(OPENVR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.deps/openvr" CACHE PATH "Path to the OpenVR source tree")

if(NOT EXISTS "${OPENVR_ROOT}/headers/openvr_driver.h")
    message(FATAL_ERROR "OpenVR headers not found at ${OPENVR_ROOT}. Clone ValveSoftware/openvr into .deps/openvr first.")
endif()

if(NOT PLATFORM)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(PLATFORM 64)
    else()
        set(PLATFORM 32)
    endif()
endif()

if(NOT WIN32)
    message(FATAL_ERROR "The AI headless HMD driver currently supports Windows builds only.")
endif()

add_definitions(-D_WIN32)
set(ARCH_TARGET "win${PLATFORM}")
set(OPENVR_INCLUDE_DIR "${OPENVR_ROOT}/headers")

find_library(
    OPENVR_LIBRARIES
    NAMES openvr_api
    PATHS
        "${OPENVR_ROOT}/lib"
        "${OPENVR_ROOT}/bin"
    PATH_SUFFIXES
        "win${PLATFORM}"
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

if(NOT OPENVR_LIBRARIES)
    message(FATAL_ERROR "openvr_api library not found under ${OPENVR_ROOT}.")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/output")

add_library(
    util_driverlog STATIC
    "${OPENVR_ROOT}/samples/drivers/utils/driverlog/driverlog.cpp"
    "${OPENVR_ROOT}/samples/drivers/utils/driverlog/driverlog.h"
)
target_include_directories(
    util_driverlog
    PUBLIC
        "${OPENVR_ROOT}/samples/drivers/utils/driverlog"
        "${OPENVR_INCLUDE_DIR}"
)
target_link_libraries(util_driverlog PRIVATE "${OPENVR_LIBRARIES}")

add_library(
    util_vrmath INTERFACE
    "${OPENVR_ROOT}/samples/drivers/utils/vrmath/vrmath.h"
)
target_include_directories(
    util_vrmath
    INTERFACE
        "${OPENVR_ROOT}/samples/drivers/utils/vrmath"
        "${OPENVR_INCLUDE_DIR}"
)
target_link_libraries(util_vrmath INTERFACE "${OPENVR_LIBRARIES}")

add_library(
    ${DRIVER_NAME} SHARED
    src/hmd_driver_factory.cpp
    src/device_provider.h
    src/device_provider.cpp
    src/hmd_device_driver.h
    src/hmd_device_driver.cpp
)

set_target_properties(
    ${DRIVER_NAME}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET_NAME}/bin/${ARCH_TARGET}>
)

target_link_libraries(${DRIVER_NAME} PRIVATE "${OPENVR_LIBRARIES}" util_driverlog util_vrmath ws2_32)
target_include_directories(${DRIVER_NAME} PRIVATE "${OPENVR_INCLUDE_DIR}")

add_custom_command(
    TARGET ${DRIVER_NAME}
    PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}"
    "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET_NAME}"
)
