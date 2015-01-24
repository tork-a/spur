cmake_minimum_required(VERSION 2.8.3)
project(spur_controller)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch nodes config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)


