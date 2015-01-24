cmake_minimum_required(VERSION 2.8.3)
project(spur_description)

find_package(catkin REQUIRED COMPONENTS urdf xacro)

catkin_package()

install(DIRECTORY launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)


