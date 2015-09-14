cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME droneObstacleDistanceCalculatorROSModule)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)



# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries




set(DRONE_OBSTACLE_DISTANCE_CALCULATOR_SOURCE_DIR
	src/sources)
	
set(DRONE_OBSTACLE_DISTANCE_CALCULATOR_INCLUDE_DIR
	src/include
	)

set(DRONE_OBSTACLE_DISTANCE_CALCULATOR_SOURCE_FILES
	
	src/sources/droneObstacleDistanceCalculatorROSModule.cpp

	
	)
	
set(DRONE_OBSTACLE_DISTANCE_CALCULATOR_HEADER_FILES

	#General
	src/include/droneObstacleDistanceCalculatorROSModule.h

	
	
	)
	
	
	
find_package(catkin REQUIRED
                COMPONENTS roscpp droneMsgsROS droneModuleROS LEVMAR)


catkin_package(
        CATKIN_DEPENDS roscpp droneMsgsROS droneModuleROS LEVMAR
  )




include_directories(${DRONE_OBSTACLE_DISTANCE_CALCULATOR_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})




add_library(droneObstacleDistanceCalculatorROSModule ${DRONE_OBSTACLE_DISTANCE_CALCULATOR_SOURCE_FILES} ${DRONE_OBSTACLE_DISTANCE_CALCULATOR_HEADER_FILES})
add_dependencies(droneObstacleDistanceCalculatorROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(droneObstacleDistanceCalculatorROSModule ${catkin_LIBRARIES})


add_executable(droneObstacleDistanceCalculatorROSModuleNode src/sources/droneObstacleDistanceCalculatorROSModuleNode.cpp)
add_dependencies(droneObstacleDistanceCalculatorROSModuleNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(droneObstacleDistanceCalculatorROSModuleNode droneObstacleDistanceCalculatorROSModule)
target_link_libraries(droneObstacleDistanceCalculatorROSModuleNode ${catkin_LIBRARIES})

