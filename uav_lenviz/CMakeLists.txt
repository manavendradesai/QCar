cmake_minimum_required(VERSION 3.5)
project(uav_lenviz)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set flags
set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Threads)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include Cpp "include" directory
include_directories(include/uav_lenviz)

# Create Cpp executable
add_executable(drone_operation src/drone_operation.cpp)
add_executable(mission_plan src/mission_plan.cpp)
add_executable(survey src/survey.cpp)
add_executable(mux_drone_pos src/mux_drone_pos.cpp)
add_executable(test_params_rclcpp src/test_params_rclcpp.cpp)

ament_target_dependencies(drone_operation rclcpp)
ament_target_dependencies(mission_plan rclcpp std_msgs)
ament_target_dependencies(survey rclcpp std_msgs geometry_msgs)
ament_target_dependencies(mux_drone_pos rclcpp std_msgs geometry_msgs)
ament_target_dependencies(test_params_rclcpp rclcpp rcl_interfaces)

target_link_libraries(mission_plan Threads::Threads)

# Install Cpp executables
install(TARGETS
  drone_operation
  mission_plan
  survey
  mux_drone_pos
  test_params_rclcpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch folder
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
