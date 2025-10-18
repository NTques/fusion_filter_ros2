cmake_minimum_required(VERSION 3.8)
project(fusion_filter_ros2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(EXECUTABLE_NAME fusion_filter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)
add_subdirectory(external/Fusion)
add_executable(${EXECUTABLE_NAME} src/main.cpp
        src/fusion_filter_node.cpp)

target_include_directories(${EXECUTABLE_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${EXECUTABLE_NAME}
        Fusion
)
ament_target_dependencies(${EXECUTABLE_NAME}
        rclcpp
        sensor_msgs
)

install(TARGETS ${EXECUTABLE_NAME}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
