cmake_minimum_required(VERSION 3.0.2)
project(mesh_conversions)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  rosservice
  )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


add_service_files(
  FILES
  ConvertMesh.srv
)

generate_messages()


catkin_package(
  CATKIN_DEPENDS message_runtime rospy rosservice
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/convert_mesh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

#install(DIRECTORY
#  config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  )
