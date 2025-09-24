cmake_minimum_required(VERSION 3.8)
project(uwb_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(example_interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${serial_INCLUDE_DIRS})
link_directories(${serial_LIBRARY_DIRS})

add_executable(uwb_node src/uwb_node.cpp)
ament_target_dependencies(uwb_node rclcpp std_msgs serial example_interfaces)

add_executable(uwb_rcv src/UWBReceiver.cpp)
ament_target_dependencies(uwb_rcv rclcpp std_msgs serial example_interfaces)

add_executable(uwb_tf src/uwb_transform.cpp)
ament_target_dependencies(uwb_tf rclcpp std_msgs nav_msgs tf2_ros serial Eigen3 example_interfaces)

add_executable(dynamic_tf_pub src/dynamic_transforms_pub.cpp)
ament_target_dependencies(dynamic_tf_pub rclcpp std_msgs nav_msgs tf2_ros serial)

target_link_libraries(uwb_node serial)
target_link_libraries(uwb_rcv serial)
target_link_libraries(uwb_tf serial)
target_link_libraries(dynamic_tf_pub serial)

install(TARGETS uwb_node
DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS uwb_rcv
DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS uwb_tf
DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS dynamic_tf_pub
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

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
