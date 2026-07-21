cmake_minimum_required(VERSION 3.20)

project(apriltag_extractor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# projectaria_tools's own CMakeLists.txt compiles everything with these two
# defines (see EIGEN_DONT_VECTORIZE/EIGEN_DONT_ALIGN at the top of
# ${ARIA_ROOT}/CMakeLists.txt) to sidestep an AVX alignment issue. That
# only applies to files compiled *inside* their add_subdirectory() scope,
# not ours -- but our code passes Eigen types across that boundary
# (Sophus::SE3d contains Eigen internals, getFocalLengths() returns
# Eigen::Vector2d by value, ...), so if our object files were compiled
# without these same defines, we'd get silent struct-layout mismatches
# when Eigen objects cross the boundary. Setting this at our own top level
# too keeps both sides binary-compatible.
add_compile_definitions(
    EIGEN_DONT_VECTORIZE
    EIGEN_DONT_ALIGN
)

# -----------------------
# Aria SDK
# -----------------------
#
# projectaria_tools doesn't install anything and doesn't ship a
# pkg-config file -- it's meant to be consumed via add_subdirectory(),
# which is what actually creates its ~30 internal static-library targets
# and resolves their (sometimes circular, e.g. sensor_calibration <->
# device_calibration via calibration_distort) dependencies correctly via
# CMake's own linking logic. include_directories()/link_directories()
# alone (the previous version of this file) never tells the linker which
# specific .a files to pull in, which is exactly why every symbol came
# back as an undefined reference.
if(NOT DEFINED ARIA_ROOT)
    message(FATAL_ERROR "You must specify ARIA_ROOT: cmake -DARIA_ROOT=/path/to/projectaria_tools ..")
endif()

if(NOT EXISTS ${ARIA_ROOT})
    message(FATAL_ERROR "projectaria_tools not found at: ${ARIA_ROOT}")
endif()

message(STATUS "Using projectaria_tools from: ${ARIA_ROOT}")

set(FETCHCONTENT_BASE_DIR ${ARIA_ROOT}/build/_deps CACHE PATH "" FORCE)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)

# Your existing ${ARIA_ROOT}/build already has Eigen/Sophus/Ocean/Dispenso
# etc. downloaded via FetchContent. Pointing FETCHCONTENT_BASE_DIR at that
# existing _deps folder means add_subdirectory() below reuses those
# sources instead of re-downloading everything into a fresh location --
# it still needs to (re)compile into the new binary dir, but it won't hit
# the network again for anything already fetched.
set(FETCHCONTENT_BASE_DIR ${ARIA_ROOT}/build/_deps CACHE PATH "" FORCE)

# Sophus (and possibly other FetchContent deps pulled in by
# projectaria_tools) declares `cmake_minimum_required(VERSION ...)` with a
# value too old for current CMake to accept at all ("Compatibility with
# CMake < 3.5 has been removed"). This tells CMake to evaluate those old
# CMakeLists.txt files as if policies were set to at least 3.5 instead of
# refusing to configure them -- it's the workaround CMake's own error
# message suggests.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)

add_subdirectory(${ARIA_ROOT} ${CMAKE_BINARY_DIR}/_aria_tools)

# -----------------------
# Other dependencies
# -----------------------
#
# Note: OpenCV and ffmpeg were only needed by the old mp4-decoder version
# of this project. Nothing under src/ includes <opencv2/...> or
# <libav*/...> anymore now that AriaReader reads frames + calibration
# straight from the .vrs, so both have been dropped here.

find_package(PkgConfig REQUIRED)
pkg_check_modules(APRILTAG REQUIRED apriltag)

# -----------------------
# Sources
# -----------------------

file(GLOB SOURCES
    src/*.cpp
)

add_executable(
    ${PROJECT_NAME}
    ${SOURCES}
)

target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
    include
    ${APRILTAG_INCLUDE_DIRS}
)

# Only the targets whose symbols our own code calls directly need to be
# listed here -- vrs_data_provider, device_calibration, sensor_calibration,
# calibration_distort, and image_debayer are all PUBLIC-linked (inside
# projectaria_tools's own CMakeLists.txt) to their own transitive
# dependencies (image, vrslib, vrs_utils, Sophus::Sophus, dispenso, ...),
# so CMake pulls all of that in automatically -- that transitive
# resolution is the whole reason to use add_subdirectory() instead of
# hand-listing every .a file ourselves.
target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
    vrs_data_provider
    device_calibration
    sensor_calibration
    calibration_distort
    image_debayer
    Eigen3::Eigen
    ${APRILTAG_LIBRARIES}
    pthread
)
