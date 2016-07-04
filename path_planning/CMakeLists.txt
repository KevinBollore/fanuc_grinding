cmake_minimum_required(VERSION 2.8.3)
project(fanuc_grinding_path_planning)

find_package(catkin REQUIRED COMPONENTS
             bezier_library
             genmsg
             geometry_msgs
             moveit_ros_planning_interface
             roscpp
             std_msgs
             tf_conversions
             visualization_msgs)

find_package(PCL 1.8.0 REQUIRED)
find_package(VTK 7.0 REQUIRED COMPONENTS vtkFiltersHybrid NO_MODULE)

add_service_files(FILES PathPlanningService.srv)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS
               bezier_library
               message_runtime
               std_msgs)

include_directories(${VTK_USE_FILE} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS} ${VTK_DEFINITIONS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/path_planning.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11")
if(VTK_LIBRARIES) # May never happen because we expect VTK 7.0 minimum
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} bezier_library bezier_grinding_surfacing)
else()
  target_link_libraries (${PROJECT_NAME} ${catkin_LIBRARIES} vtkHybrid vtkWidgets ${PCL_LIBRARIES} bezier_library bezier_grinding_surfacing)
endif()

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
