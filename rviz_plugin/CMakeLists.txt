cmake_minimum_required(VERSION 2.8.3)
project(fanuc_grinding_rviz_plugin)

find_package(catkin REQUIRED COMPONENTS
                    fanuc_grinding_alignment
                    fanuc_grinding_comparison
                    fanuc_grinding_path_planning
                    fanuc_grinding_post_processor
                    fanuc_grinding_scanning
                    rviz)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  src/alignment_widget.h
  src/comparison_widget.h
  src/${PROJECT_NAME}.h
  src/path_planning_widget.h
  src/post_processor_widget.h
  src/scanning_widget.h
)

set(SOURCE_FILES
  src/alignment_widget.cpp
  src/comparison_widget.cpp
  src/${PROJECT_NAME}.cpp
  src/path_planning_widget.cpp
  src/post_processor_widget.cpp
  src/scanning_widget.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11")
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} fanuc_grinding_scanning_generate_messages_cpp)

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

