cmake_minimum_required(VERSION 3.8)
project(stm32)

# 1. Standard Compiler Flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bolide_interfaces REQUIRED)

# 3. Define Executables
add_executable(stm32_node src/stm32_node.cpp)
add_executable(cmd_vel_node src/cmd_vel_node.cpp)

# 4. Manage Dependencies
# Grouped together for clarity
set(dependencies
        rclcpp
        std_msgs
        sensor_msgs
        bolide_interfaces
)

ament_target_dependencies(stm32_node ${dependencies})
ament_target_dependencies(cmd_vel_node ${dependencies})

# 5. Include Directories & Features
# Applied to both nodes so cmd_vel_node can find your headers too
target_include_directories(stm32_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(cmd_vel_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(stm32_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(cmd_vel_node PUBLIC c_std_99 cxx_std_17)

# 6. Install Logic
# Added cmd_vel_node to the install list so 'ros2 run' can find it
install(TARGETS
        stm32_node
        cmd_vel_node
        DESTINATION lib/${PROJECT_NAME})

# Install include directory (optional but recommended for libraries)
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME})

# 7. Testing & Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 8. Package Export (MUST be last)
ament_package()