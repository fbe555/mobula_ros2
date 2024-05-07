cmake_minimum_required(VERSION 3.8)
project(bluerov2_nmpc)


set(ROS_BUILD_TYPE Debug)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.0.1 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# Message generation
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/PwmCmd.msg
 )
ament_export_dependencies(rosidl_default_runtime)


# Add library
add_library(lib${PROJECT_NAME} SHARED
  solver/qpoases/SRC/Bounds.cpp
  solver/qpoases/SRC/Constraints.cpp
  solver/qpoases/SRC/CyclingManager.cpp
  solver/qpoases/SRC/Indexlist.cpp
  solver/qpoases/SRC/MessageHandling.cpp
  solver/qpoases/SRC/QProblem.cpp
  solver/qpoases/SRC/QProblemB.cpp
  solver/qpoases/SRC/SubjectTo.cpp
  solver/qpoases/SRC/Utils.cpp
  solver/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  solver/nmpc_qpoases_interface.cpp
  solver/nmpc_integrator.c
  solver/nmpc_solver.c
  solver/nmpc_auxiliary_functions.c
)

target_compile_features(lib${PROJECT_NAME} PRIVATE c_std_99 cxx_std_17) 

target_include_directories(lib${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/solver>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/solver/qpoases>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/solver/qpoases/INCLUDE>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/solver/qpoases/INCLUDE/EXTRAS>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/solver/qpoases/SRC>
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS lib${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Add node executables
add_executable(${PROJECT_NAME}_node 
  src/nmpc_bluerov2_2D_main.cpp 
  src/nmpc_bluerov2_2D.cpp 
)

ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
  rclcpp
  tf2
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  
) 

target_link_libraries(${PROJECT_NAME}_node PRIVATE
  lib${PROJECT_NAME}
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/thrust_allocator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
