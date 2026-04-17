cmake_minimum_required(VERSION 3.8)
project(double_steering_drive_controller NONE)

#
# Binary-only (.so) distribution of the double_steering_drive_controller plugin.
#
# This package does NOT compile anything. It only copies a pre-built shared
# library, its public headers, and the pluginlib description XML into the
# standard install layout so that ros2_control (controller_manager) can
# discover and dlopen() the plugin at runtime.
#
# The pre-built library is shipped as a GitHub Release asset and placed under
# libs/double_steering_drive_controller/lib/libdouble_steering_drive_controller.so
# before running `colcon build`.  See README.md for the fetch instructions and
# the SHA-256 of the official binary.
#

find_package(ament_cmake REQUIRED)

set(LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/libdouble_steering_drive_controller.so")
set(PLUGIN_XML "${CMAKE_CURRENT_SOURCE_DIR}/double_steering_drive_plugin.xml")

if(NOT EXISTS "${LIB_PATH}")
  message(FATAL_ERROR
    "Missing pre-built binary: ${LIB_PATH}\n"
    "Download it from the GitHub Release page of this repository\n"
    "and place it at libs/double_steering_drive_controller/lib/ before running colcon build.\n"
    "See libs/double_steering_drive_controller/README.md for details.")
endif()

# Install the pre-built .so into the ament install lib/ directory.
install(
  FILES "${LIB_PATH}"
  DESTINATION lib
)

# Install the pluginlib description XML so pluginlib_export_plugin_description_file
# (declared in package.xml <export>) resolves it.
install(
  FILES "${PLUGIN_XML}"
  DESTINATION share/${PROJECT_NAME}
)

# Install the public header(s), if provided, so downstream packages that #include
# them can still link/compile against this package.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")
  install(
    DIRECTORY include/
    DESTINATION include
  )
endif()

ament_package()
