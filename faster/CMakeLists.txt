cmake_minimum_required(VERSION 2.8.3)
project(faster)



## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
#add_definitions(-std=c99)

#find_program(iwyu_path NAMES /home/jtorde/iwyu/build/include-what-you-use)

#if(NOT iwyu_path)
#  message(FATAL_ERROR "Could not find the program include-what-you-use")
#endif()

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#In the ws/ directory there must be the file FindGUROBI.cmake (available https://github.com/joschu/trajopt/blob/master/cmake/modules/FindGUROBI.cmake ). I had to modify it a little bit. 
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")
find_package(GUROBI REQUIRED)
find_package(NLOPT REQUIRED)
find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})

if(GUROBI_FOUND)
  message(STATUS "GUROBI FOUND")
else(GUROBI_FOUND)
  message(FATAL_ERROR "GUROBI NOT FOUND!")
endif(GUROBI_FOUND)


set(CMAKE_BUILD_TYPE RelWithDebInfo) #From terminal: catkin config -DCMAKE_BUILD_TYPE=Release (or RelWithDebInfo) or Degub

include_directories(${GUROBI_INCLUDE_DIRS})
#set(GUROBI_LIBRARIES "${GUROBI_CXX_LIBRARY};${GUROBI_LIBRARY}" )


find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	snapstack_msgs
	faster_msgs
	sensor_msgs
	pcl_ros
	decomp_util
	decomp_ros_utils
	rviz_visual_tools
	separator
)
find_package(decomp_util REQUIRED)
include_directories(${DECOMP_UTIL_INCLUDE_DIRS})

find_package(jps3d REQUIRED)
include_directories(${JPS3D_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cvx
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;$ENV{GUROBI_HOME}/lib/libgurobi81.so;$ENV{GUROBI_HOME}/lib/" )


add_library(utils STATIC src/utils.cpp)
#set_property(TARGET utils PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
target_include_directories (utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
#target_precompile_headers(utils PUBLIC src/utils.hpp)
target_link_libraries(utils PUBLIC ${PCL_LIBRARIES})


add_library(solverNlopt STATIC src/solvers/nlopt/solverNlopt.cpp src/solvers/nlopt/solverNlopt.hpp)
#set_property(TARGET solverNlopt PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
target_include_directories (solverNlopt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${DECOMP_UTIL_INCLUDE_DIRS})
target_link_libraries(solverNlopt PUBLIC ${NLOPT_LIBRARIES} ${DECOMP_UTIL_LIBRARIES} utils ${catkin_LIBRARIES})


add_library(jps_manager STATIC src/jps_manager.cpp src/jps_manager.hpp)
target_include_directories (jps_manager PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(jps_manager PUBLIC ${JPS3D_LIBRARIES} ${DECOMP_UTIL_LIBRARIES})

add_library(solverGurobi STATIC src/solvers/solverGurobi.cpp src/solvers/solverGurobi.hpp)
#set_property(TARGET solverGurobi PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
target_include_directories (solverGurobi PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(solverGurobi PUBLIC ${GUROBI_LIBRARIES})


add_library(faster_hpp STATIC src/faster.hpp)
target_include_directories (faster_hpp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(faster_hpp PUBLIC ${CGAL_LIBS} solverGurobi jps_manager solverNlopt)


add_library(faster STATIC src/faster.cpp)
#set_property(TARGET faster PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
target_include_directories (faster PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(faster PUBLIC faster_hpp)


add_library(faster_ros STATIC src/faster_ros.cpp src/faster_ros.hpp)
#set_property(TARGET faster_ros PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
target_include_directories (faster_ros PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(faster_ros PUBLIC ${catkin_LIBRARIES} faster)


add_executable(${PROJECT_NAME}_node src/main.cpp)

#IF CVXGEN and GUROBI are required:
#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${JPS3D_LIBRARIES} ${DECOMP_UTIL_LIBRARIES} ${CMAKE_CURRENT_LIST_DIR}/src/solvers/cvxgen/libcvx_vel.so ${CMAKE_CURRENT_LIST_DIR}/src/solvers/cvxgen/libcvx_accel.so ${CMAKE_CURRENT_LIST_DIR}/src/solvers/cvxgen/libcvx_jerk.so ${GUROBI_LIBRARIES})

#IF only GUROBI is required:
#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${JPS3D_LIBRARIES} ${DECOMP_UTIL_LIBRARIES} ${GUROBI_LIBRARIES} )


#If GUROBI and NLOPT are required
target_link_libraries(${PROJECT_NAME}_node PUBLIC faster_ros)


add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} )


# add_executable(simplex_example src/solvers/simplex/example.cpp)
# add_dependencies(simplex_example ${catkin_EXPORTED_TARGETS} )
# target_link_libraries(simplex_example ${catkin_LIBRARIES})


#add_executable(${PROJECT_NAME}_node_novale src/main_novale.cpp)
#add_dependencies(${PROJECT_NAME}_node_novale ${catkin_EXPORTED_TARGETS} )
#target_link_libraries(${PROJECT_NAME}_node_novale ${catkin_LIBRARIES} ${PCL_LIBRARIES})



#add_executable(${PROJECT_NAME}_sample src/solvers/nlopt/main.cpp src/solvers/nlopt/solverNlopt.cpp)
#target_link_libraries(${PROJECT_NAME}_sample  ${NLOPT_LIBRARIES}  ${CGAL_LIBS})
#add_dependencies(${PROJECT_NAME}_sample ${catkin_EXPORTED_TARGETS} )


#add_executable(gurobi_continuous_exec gurobi_continuous.cpp)
#target_link_libraries(gurobi_continuous_exec ${GUROBI_LIBRARIES})

#add_executable(gurobi_discrete_exec gurobi_discrete.cpp)
#target_link_libraries(gurobi_discrete_exec ${GUROBI_LIBRARIES} )



#add_executable(solverGurobi_exec src/solvers/solverGurobi.cpp)
#target_link_libraries(solverGurobi_exec ${GUROBI_LIBRARIES})
#add_executable(${PROJECT_NAME}_deubg_node src/debug.cpp)
#target_link_libraries(${PROJECT_NAME}_deubg_node ${catkin_LIBRARIES} ${CMAKE_CURRENT_LIST_DIR}/src/libcvx.so)
#add_dependencies(${PROJECT_NAME}_deubg_node ${catkin_EXPORTED_TARGETS})


