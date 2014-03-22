cmake_minimum_required(VERSION 2.8.3)
project(aiciss_logger)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    std_msgs
    nav_msgs
    message_filters
		tf
)

catkin_package()

add_executable(aiciss_logger_node 
  ros/src/aiciss_logger_node.cpp
)
add_dependencies(aiciss_logger_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(aiciss_logger_node
  ${catkin_LIBRARIES}
)

add_executable(publish_base_commands_node
  ros/src/publish_base_commands_node.cpp
)
add_dependencies(publish_base_commands_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_base_commands_node
  ${catkin_LIBRARIES}
)

