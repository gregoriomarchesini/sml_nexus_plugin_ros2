cmake_minimum_required(VERSION 3.8)
project(sml_nexus_plugin_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(rcutils)
find_package(std_msgs      REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros    REQUIRED)
find_package(gazebo        REQUIRED)
find_package(nav_msgs      REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


# directories used by rclcpp package
include_directories(include ${rclcpp_INCLUDE_DIRS})

add_library(sml_nexus_ros2_force_based_move 
            SHARED src/sml_nexus_ros2_force_based_move.cpp)

target_include_directories(sml_nexus_ros2_force_based_move PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# specify the dependancies
ament_target_dependencies(sml_nexus_ros2_force_based_move
                          "gazebo_ros"
                          "rclcpp"
                          "geometry_msgs"
                          "nav_msgs"
                          "tf2"
                          "tf2_geometry_msgs"
                          "tf2_ros")


ament_export_libraries(sml_nexus_ros2_force_based_move)
# This command makes sure that the gazebo plug in is available in the lib and bin folder in the build directory
install(
    TARGETS sml_nexus_ros2_force_based_move
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)



# If you specify 
ament_package()