cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)


project(des_mubot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros
  std_msgs
  roscpp
  roslaunch
)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS
    DEPENDS
    rospy
    gazebo_ros
    std_msgs
    roscpp
    roslaunch
)

catkin_python_setup(
)

catkin_install_python(PROGRAMS
  scripts/gazebo_connection.py
  scripts/GA_Algorithm.py
  scripts/start_GA_vary.py
  scripts/continue_GA_vary.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
#AN2
add_library(actuator SHARED actuator.cc)
target_link_libraries(actuator ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
add_library(Hydro SHARED Hydro.cc)
target_link_libraries(Hydro ${GAZEBO_LIBRARIES})

