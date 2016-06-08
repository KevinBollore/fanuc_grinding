cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg tf_conversions moveit_ros_planning_interface visualization_msgs bezier_library geometry_msgs)
find_package(PCL 1.8.0 REQUIRED)
find_package(VTK 7.0 REQUIRED COMPONENTS vtkFiltersHybrid NO_MODULE)

add_service_files(FILES PathPlanningService.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS bezier_library)

include_directories(${VTK_USE_FILE} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS} ${VTK_DEFINITIONS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(path_planning src/path_planning.cpp)
set_property(TARGET path_planning APPEND_STRING PROPERTY COMPILE_FLAGS -Wall)
if(VTK_LIBRARIES) # May never happen because we expect VTK 7.0 minimum
  target_link_libraries(path_planning ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} bezier_library)
else()
  target_link_libraries (path_planning ${catkin_LIBRARIES} vtkHybrid vtkWidgets ${PCL_LIBRARIES} bezier_library)
endif()

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
