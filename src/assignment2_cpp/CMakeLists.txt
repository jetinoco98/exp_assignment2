cmake_minimum_required(VERSION 3.5)
project(assignment2_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(interfaces REQUIRED) 

find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  gazebo_msgs
  interfaces
  plansys2_msgs
  nav2_msgs
  plansys2_domain_expert
  plansys2_executor
  plansys2_planner
  plansys2_problem_expert
  plansys2_pddl_parser
)

# EXECUTABLES WITH DEPENDENCIES
add_executable(move_action src/move_action.cpp)
ament_target_dependencies(move_action ${dependencies})

add_executable(inspect_action src/inspect_action.cpp)
ament_target_dependencies(inspect_action ${dependencies})

# add_executable(planner_controller src/planner_controller.cpp)
# ament_target_dependencies(planner_controller ${dependencies})

# INSTALL DIRECTORIES
install(DIRECTORY launch pddl 
  DESTINATION share/${PROJECT_NAME})

# INSTALL TARGETS
install(TARGETS
  move_action
  inspect_action
  # planner_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()

