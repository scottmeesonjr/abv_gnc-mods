cmake_minimum_required(VERSION 3.14)
project(abv_rl-cpp CXX)

# ── C++ standard ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS 2 deps ────────────────────────────────────────────────────────────────
find_package(ament_cmake  REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(abv_msgs     REQUIRED)
find_package(abv_common   REQUIRED)

# ── ONNX Runtime ──────────────────────────────────────────────────────────────
# Set ORT_ROOT to your local ONNX Runtime install if it isn't on the system path.
# e.g.:  cmake -DORT_ROOT=/opt/onnxruntime ..
#
# Expected layout under ORT_ROOT:
#   include/onnxruntime/onnxruntime_cxx_api.h
#   lib/libonnxruntime.so  (or .dylib / .dll)

if(DEFINED ORT_ROOT)
    set(ORT_INCLUDE_DIR "${ORT_ROOT}/include")
    set(ORT_LIB_DIR     "${ORT_ROOT}/lib")
else()
    # Fall back to pkg-config / system paths
    find_path(ORT_INCLUDE_DIR
        NAMES onnxruntime/onnxruntime_cxx_api.h
        PATHS /usr/local/include /usr/include
    )
    find_library(ORT_LIB
        NAMES onnxruntime
        PATHS /usr/local/lib /usr/lib
    )
endif()

if(NOT ORT_LIB)
    find_library(ORT_LIB NAMES onnxruntime PATHS "${ORT_LIB_DIR}")
endif()

if(NOT ORT_INCLUDE_DIR OR NOT ORT_LIB)
    message(FATAL_ERROR
        "ONNX Runtime not found. Install it and set -DORT_ROOT=<path>.")
endif()

message(STATUS "ORT include : ${ORT_INCLUDE_DIR}")
message(STATUS "ORT lib     : ${ORT_LIB}")

# ── Target ────────────────────────────────────────────────────────────────────
add_executable(${PROJECT_NAME}
    src/rl_policy_node.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/include
       	${ORT_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    ${ORT_LIB}
    abv_common::abv_common
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    abv_msgs
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
