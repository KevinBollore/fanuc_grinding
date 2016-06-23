cmake_minimum_required(VERSION 2.8.3)
project(fanuc_grinding_post_processor)

find_package(catkin REQUIRED COMPONENTS
             eigen_conversions
             fanuc_post_processor_library
             genmsg
             geometry_msgs
			 roscpp
			 std_msgs
			 tf
			 tf_conversions)

add_service_files(FILES PostProcessorService.srv)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS
               fanuc_post_processor_library
               geometry_msgs
               message_runtime
               std_msgs)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/post_processor.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11")
target_link_libraries (${PROJECT_NAME} ${catkin_LIBRARIES} fanuc_post_processor_library curl curlite)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
