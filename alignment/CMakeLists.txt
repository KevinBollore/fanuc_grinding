cmake_minimum_required(VERSION 2.8.3)
project(alignment)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg tf_conversions moveit_ros_planning_interface visualization_msgs)
find_package(PCL 1.8.0 REQUIRED)
find_package(VTK 7.0 REQUIRED)
include(${VTK_USE_FILE})

add_service_files(FILES AlignmentService.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(${VTK_USE_FILE} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${VTK_USE_FILE} ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(alignment src/alignment.cpp)
target_link_libraries(alignment ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
