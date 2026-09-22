cmake_minimum_required(VERSION 3.8)
project(prestobot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Use PkgConfig to find libserial since it doesn't provide its own CMake config files.
find_package(PkgConfig REQUIRED) # Make PkgConfig required
pkg_check_modules(LIBSERIAL REQUIRED libserial) # Check for libserial using pkg-config
# The following line is causing the error because libserial doesn't provide a .cmake config file
# find_package(libserial REQUIRED) # <-- REMOVE or COMMENT OUT THIS LINE

include_directories(include)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/prestobot_interface.cpp
)

# Use the variables provided by pkg_check_modules for include directories and link libraries
target_include_directories(${PROJECT_NAME} PRIVATE 
  include
  "${LIBSERIAL_INCLUDE_DIRS}" # Use LIBSERIAL_INCLUDE_DIRS
)
target_link_libraries(${PROJECT_NAME}
  "${LIBSERIAL_LIBRARIES}") # Use LIBSERIAL_LIBRARIES
# The previous target_link_libraries for LibSerial_LIBRARIES needs to be removed/commented out
# target_link_libraries(${PROJECT_NAME} ${LibSerial_LIBRARIES}) # REMOVE or COMMENT OUT THIS LINE

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface prestobot_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()