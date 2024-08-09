cmake_minimum_required(VERSION 3.8)
project(gnss_preprocessing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add executables
add_executable(gnss_preprocessing_component
  src/gnss_preprocessing_component.cpp
)

add_executable(gnss_preprocessing
  src/gnss_preprocessing.cpp
  src/gnss_preprocessing_core.cpp  # gnss_preprocessing_core.cppをリンク
)

add_executable(save_gnss_path src/save_gnss_path.cpp)

# Include directories
target_include_directories(gnss_preprocessing_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(gnss_preprocessing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(gnss_preprocessing_component
  rclcpp
  tf2
  tf2_ros
  nav_msgs
  geometry_msgs
  sensor_msgs
  Eigen3
)

ament_target_dependencies(gnss_preprocessing
  rclcpp
  tf2
  tf2_ros
  nav_msgs
  geometry_msgs
  sensor_msgs
  Eigen3
)

ament_target_dependencies(save_gnss_path rclcpp geometry_msgs)

# Link libraries
target_link_libraries(gnss_preprocessing_component
  rclcpp::rclcpp
  Eigen3::Eigen
)

target_link_libraries(gnss_preprocessing
  rclcpp::rclcpp
  Eigen3::Eigen
)

# Install executables
install(TARGETS
  gnss_preprocessing_component
  gnss_preprocessing
  save_gnss_path
  DESTINATION lib/${PROJECT_NAME})

# Install other files
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

