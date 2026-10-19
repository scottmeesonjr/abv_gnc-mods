cmake_minimum_required(VERSION 3.8)
project(abv_guidance)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(abv_msgs REQUIRED)
find_package(abv_common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(plog REQUIRED)

set(ABV_GUIDANCE_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/CollisionAvoidPathGenerator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/FromFileGenerator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/PathWatchdog.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/RosCommandSource.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Scene.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/StateMachine.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/StraightLineGenerator.cpp
)

# Create executable
add_executable(${PROJECT_NAME} ${ABV_GUIDANCE_SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)

# link libraries 
target_link_libraries(${PROJECT_NAME} abv_common::abv_common yaml-cpp plog::plog)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    abv_msgs
    Eigen3
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()