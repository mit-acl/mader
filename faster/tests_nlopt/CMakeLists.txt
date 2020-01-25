cmake_minimum_required(VERSION 2.8.3)
project(tests_nlopt)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#In the ws/ directory there must be the file FindGUROBI.cmake (available https://github.com/joschu/trajopt/blob/master/cmake/modules/FindGUROBI.cmake ). I had to modify it a little bit. 
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")
find_package(NLOPT REQUIRED)



set(CMAKE_BUILD_TYPE RelWithDebInfo) #From terminal: catkin config -DCMAKE_BUILD_TYPE=Release (or RelWithDebInfo)


find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	sensor_msgs
	pcl_ros
)


find_package(jps3d REQUIRED)
include_directories(${JPS3D_INCLUDE_DIRS})

catkin_package(

)


include_directories(${catkin_INCLUDE_DIRS})


#add_executable(${PROJECT_NAME}_node src/main.cpp src/faster.cpp src/faster_ros.cpp src/utils.cpp  src/jps_manager.cpp src/solvers/solverGurobi.cpp)
#IF only GUROBI is required:
#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${JPS3D_LIBRARIES} ${DECOMP_UTIL_LIBRARIES} ${GUROBI_LIBRARIES})
#add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} )


#add_executable(${PROJECT_NAME}_node_novale src/main_novale.cpp)
#add_dependencies(${PROJECT_NAME}_node_novale ${catkin_EXPORTED_TARGETS} )
#target_link_libraries(${PROJECT_NAME}_node_novale ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(main_exec src/main.cpp src/solverNlopt.cpp)
add_dependencies(main_exec ${catkin_EXPORTED_TARGETS} )
target_link_libraries(main_exec ${NLOPT_LIBRARIES})


add_executable(example_exec src/example.cpp)
add_dependencies(example_exec ${catkin_EXPORTED_TARGETS} )
target_link_libraries(example_exec ${NLOPT_LIBRARIES})







