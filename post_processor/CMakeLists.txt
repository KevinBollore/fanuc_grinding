cmake_minimum_required(VERSION 2.8.3)
project(post_processor)

find_package(catkin REQUIRED COMPONENTS
			 roscpp
			 std_msgs
			 genmsg
			 tf_conversions
			 moveit_ros_planning_interface
			 visualization_msgs
			 geometry_msgs
			 fanuc_post_processor_library)

#find_package(CURL)

add_service_files(FILES PostProcessorService.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS fanuc_post_processor_library)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(post_processor src/post_processor.cpp)
set_property(TARGET post_processor APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11")
target_link_libraries (post_processor ${catkin_LIBRARIES} fanuc_post_processor_library curl curlite)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
