cmake_minimum_required(VERSION 3.0.2)

project(taurasim)

find_package(catkin REQUIRED)

catkin_package()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  gazebo_msgs
)

foreach(dir config launch meshes urdf media worlds models)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
