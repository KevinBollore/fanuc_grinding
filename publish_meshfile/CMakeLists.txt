cmake_minimum_required(VERSION 2.8.3)
project(publish_meshfile)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg tf_conversions moveit_ros_planning_interface visualization_msgs)

find_package(PCL 1.8.0 REQUIRED)

add_service_files(FILES PublishMeshfileService.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(publish_meshfile src/publish_meshfile.cpp)
target_link_libraries(publish_meshfile ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
