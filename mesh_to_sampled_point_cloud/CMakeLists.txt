cmake_minimum_required(VERSION 3.1)
project(mesh_to_sampled_point_cloud)


set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  pcl_ros
  roscpp
  rosservice
  )

# find PCL additionally and link to libraries, as otherwise vtkPolyDataMapper cannot be found during execution
find_package(PCL REQUIRED COMPONENTS common io filters)


add_service_files(
  FILES
  ConvertMeshToSampledPointCloud.srv)


generate_messages(
  DEPENDENCIES
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  pcl_ros
  roscpp
  rosservice
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# include PCL directories
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(HEADER_FILES
  include/${PROJECT_NAME}/mesh_to_sampled_point_cloud.h
  include/${PROJECT_NAME}/pcl_mesh_sampling_extract.h
  )

set(SOURCES
  src/mesh_to_sampled_point_cloud.cpp
  src/mesh_to_sampled_point_cloud_node.cpp
  src/pcl_mesh_sampling_extract.cpp
  )


## Declare a C++ library
add_library(${PROJECT_NAME} ${HEADER_FILES} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
# additionally link to PCL and VTK libraries as otherwise vtkPolyDataMapper cannot be found during execution
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})


## Installation
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

#install(DIRECTORY
#  config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  )

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )
