cmake_minimum_required(VERSION 3.8)
project(cartpole_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(
  cartpole_physics 
  src/dynamics/cartpole.cpp 
  src/math/rk4_integration.cpp
)
target_link_libraries(cartpole_physics PUBLIC Eigen3::Eigen)
target_include_directories(cartpole_physics PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(cartpole_node src/cartpole_node.cpp)
target_link_libraries(
  cartpole_node
  cartpole_physics
)

# Require C99 and C++17
target_compile_features(cartpole_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  cartpole_node
  "rclcpp"
  "std_msgs"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS cartpole_physics
  EXPORT export_cartpole_physics
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_cartpole_physics HAS_LIBRARY_TARGET)

install(TARGETS cartpole_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_export_include_directories(include)
ament_export_libraries(cartpole_physics)

ament_package()
