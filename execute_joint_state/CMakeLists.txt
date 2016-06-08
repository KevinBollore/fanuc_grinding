cmake_minimum_required(VERSION 2.8.3)
project(execute_joint_state)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg tf_conversions moveit_ros_planning_interface visualization_msgs)

add_service_files(FILES ExecuteJointStateService.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(execute_joint_state src/execute_joint_state.cpp)
target_link_libraries(execute_joint_state ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
