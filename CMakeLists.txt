cmake_minimum_required(VERSION 3.8)
project(fusion_filter_ros2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(EXECUTABLE_NAME fusion_filter)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

generate_parameter_library(
        fusion_filter_parameters
        fusion_filter_parameters.yaml
)

include_directories(include)
add_subdirectory(external/Fusion)
add_executable(${EXECUTABLE_NAME}
        src/main.cpp
        src/fusion_filter_node.cpp
)

target_include_directories(${EXECUTABLE_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${EXECUTABLE_NAME}
        Fusion
        fusion_filter_parameters
)
ament_target_dependencies(${EXECUTABLE_NAME}
        rclcpp
        sensor_msgs
)

install(TARGETS ${EXECUTABLE_NAME} fusion_filter_parameters
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_package()

