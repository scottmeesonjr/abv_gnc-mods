cmake_minimum_required(VERSION 3.8)
project(abv_control)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(abv_msgs REQUIRED)
find_package(abv_common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(plog REQUIRED)

set(ABV_CONTROL_SOURCES
  src/ControlLawHandler.cpp
  src/main.cpp
  src/PidControlLaw.cpp
)

add_executable(${PROJECT_NAME} ${ABV_CONTROL_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/include
	${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} abv_common::abv_common yaml-cpp plog::plog)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  abv_msgs
  Eigen3
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
