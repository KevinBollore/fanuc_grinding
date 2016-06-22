cmake_minimum_required(VERSION 2.8.3)
project(fanuc_grinding_alignment)

find_package(catkin REQUIRED COMPONENTS
             genmsg
             roscpp
             std_msgs)

add_service_files(FILES AlignmentService.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
               message_runtime
               std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/alignment.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
