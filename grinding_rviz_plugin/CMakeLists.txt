cmake_minimum_required(VERSION 2.8.3)
project(grinding_rviz_plugin)

find_package(catkin REQUIRED COMPONENTS rviz alignment scanning comparison path_planning post_processor)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  src/scanning_widget.h
  src/alignment_widget.h
  src/comparison_widget.h
  src/path_planning_widget.h
  src/post_processor_widget.h
  src/grinding_rviz_plugin.h
)

set(SOURCE_FILES
  src/scanning_widget.cpp
  src/alignment_widget.cpp
  src/comparison_widget.cpp
  src/path_planning_widget.cpp
  src/post_processor_widget.cpp
  src/grinding_rviz_plugin.cpp 
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11")
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} scanning_generate_messages_cpp)

## Install rules
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

