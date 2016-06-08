cmake_minimum_required(VERSION 2.8.3)
project(alignment)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg tf_conversions moveit_ros_planning_interface visualization_msgs)

add_service_files(FILES AlignmentService.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(alignment src/alignment.cpp)
target_link_libraries(alignment ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
