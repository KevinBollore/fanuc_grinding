cmake_minimum_required(VERSION 2.8.3)
project(fanuc_grinding_scanning)

find_package(catkin REQUIRED COMPONENTS
			 fanuc_grinding_execute_joint_state
			 fanuc_grinding_publish_meshfile
			 genmsg
			 moveit_ros_planning_interface
			 roscpp
			 std_msgs
			 tf_conversions)

find_package(PCL 1.8.0 REQUIRED)

add_service_files(FILES ScanningService.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
               message_runtime
               std_msgs)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}) # Order matters!

add_executable(${PROJECT_NAME} src/scanning.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11")
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
