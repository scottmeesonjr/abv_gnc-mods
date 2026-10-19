cmake_minimum_required(VERSION 3.8)
project(abv_thrusters)

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

# Architecture detection - selects UdpThrusterDriver (x86, for
# simulation/dev) or GpioThrusterDriver (ARM, real Jetson hardware) at
# compile time in ThrusterCommander.cpp
set(ARCH_X86 FALSE)
set(ARCH_ARM FALSE)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(ARCH_X86 TRUE)
  add_definitions(-DARCH_X86)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(ARCH_ARM TRUE)
  add_definitions(-DARCH_ARM)
endif()

# Source files (common)
set(ABV_THRUSTERS_SOURCES
  src/LookupTableThrusterMapper.cpp
  src/main.cpp
  src/MatrixThrusterMapper.cpp
  src/ThrusterCommander.cpp
  src/ThrusterCommandHandler.cpp
  src/UdpThrusterDriver.cpp
)

# Conditionally include architecture dependent sources
if(ARCH_ARM)
	list(APPEND ABV_THRUSTERS_SOURCES
		src/GpioThrusterDriver.cpp)
endif()

add_executable(${PROJECT_NAME} ${ABV_THRUSTERS_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/include
	${EIGEN3_INCLUDE_DIRS}
)

if(ARCH_ARM)
	target_include_directories(${PROJECT_NAME}
    PRIVATE
      /usr/include/
  )
endif()

# Link libraries
target_link_libraries(${PROJECT_NAME} abv_common::abv_common yaml-cpp plog::plog)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  abv_msgs
  Eigen3
)

if(ARCH_ARM)
	set(jetgpio_LIB /usr/lib/libjetgpio.so) # manually set because it doesnt use cmake
  target_link_libraries(${PROJECT_NAME} ${jetgpio_LIB})
endif()

# Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
