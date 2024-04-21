cmake_minimum_required(VERSION 3.5)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# cmakelist

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(PCL REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)
  # find_package(PCL COMPONENT REQUIRED)

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

ament_package()

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cpp_code src/testnode.cpp)
target_link_libraries(cpp_code ${PCL_LIBRARIES} ${catkin_LIBRARIES})
ament_target_dependencies(cpp_code rclcpp std_msgs)

# add_executable(voxel_code src/passvoxel.cpp)
# target_link_libraries(voxel_code ${PCL_LIBRARIES})
# ament_target_dependencies(cpp_code rclcpp std_msgs)

add_executable(talker src/testpub.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/testsub.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(voxelros src/voxelros.cpp)
target_link_libraries(voxelros ${PCL_LIBRARIES})
ament_target_dependencies(voxelros rclcpp std_msgs sensor_msgs pcl_conversions)
# target_link_libraries

add_executable(clusterros src/clusterros.cpp)
target_link_libraries(clusterros ${PCL_LIBRARIES})
ament_target_dependencies(clusterros rclcpp std_msgs sensor_msgs pcl_conversions)

add_executable(lidarcluster src/lidarcluster.cpp)
target_link_libraries(lidarcluster ${PCL_LIBRARIES})
ament_target_dependencies(lidarcluster rclcpp std_msgs sensor_msgs pcl_conversions)


install(TARGETS
    talker
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    listener
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    voxelros
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    lidarcluster
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    clusterros
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    cpp_code
    DESTINATION lib/${PROJECT_NAME}
)

