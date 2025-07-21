cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)

project(end_effector_description)

# find dependencies
find_package(ament_cmake REQUIRED)

# For each subfolder, find any more installation rules.
install(
	DIRECTORY launch rviz urdf etc meshes
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
