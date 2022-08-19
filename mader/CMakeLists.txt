cmake_minimum_required(VERSION 2.8.3)
project(mader)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
#add_definitions(-std=c99)


find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_finders")


find_package(CGAL REQUIRED)


option(USE_GUROBI "Use GUROBI as the solver" ON) #if OFF, then it will use NLOPT as the solver


include(${CGAL_USE_FILE})

set(CMAKE_BUILD_TYPE Release) #From terminal: catkin config -DCMAKE_BUILD_TYPE=Release (or RelWithDebInfo) or Debug



find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	snapstack_msgs
	mader_msgs
	sensor_msgs
	decomp_util
	decomp_ros_utils
	rviz_visual_tools
	separator
  	jsk_rviz_plugins
)
find_package(decomp_util REQUIRED)
include_directories(${DECOMP_UTIL_INCLUDE_DIRS})

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)

#For precompile headers see example here
#https://gitlab.kitware.com/cmake/cmake/tree/375d01c6808713a0cfeef9ea092c8236ba063525/Tests/PrecompileHeaders

add_library(cgal_utils STATIC src/cgal_utils)
target_include_directories (cgal_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(cgal_utils PUBLIC ${DECOMP_UTIL_LIBRARIES} ${catkin_LIBRARIES})

add_library(utils STATIC src/utils.cpp)
target_include_directories (utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(utils PUBLIC ${catkin_LIBRARIES})

add_library(octopus_search STATIC src/octopus_search.cpp src/bspline_utils.cpp src/utils.cpp) 
target_include_directories (octopus_search PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(octopus_search PUBLIC ${catkin_LIBRARIES}) 

if(USE_GUROBI)

	find_package(GUROBI REQUIRED)

	if(GUROBI_FOUND)
	  message(STATUS "GUROBI FOUND")
	else(GUROBI_FOUND)
	  message(FATAL_ERROR "GUROBI NOT FOUND!")
	endif(GUROBI_FOUND)

	include_directories(${GUROBI_INCLUDE_DIRS})
	file(GLOB GurobiSOFiles $ENV{GUROBI_HOME}/lib/libgurobi*[0-9].so) #files that are start with libgurobi and end with number.so
	set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;${GurobiSOFiles}" ) #;$ENV{GUROBI_HOME}/lib/

	add_library(solver STATIC src/solver_gurobi.cpp src/bspline_utils.cpp src/solver_gurobi_utils.cpp src/solver_gurobi_guess.cpp)
	target_compile_definitions(solver PUBLIC USE_GUROBI_FLAG=1) 
	target_include_directories (solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${DECOMP_UTIL_INCLUDE_DIRS})
	target_link_libraries(solver PUBLIC ${GUROBI_LIBRARIES} ${DECOMP_UTIL_LIBRARIES} utils octopus_search ${catkin_LIBRARIES} cgal_utils)

	add_executable(test_gurobi src/examples/test_gurobi.cpp)
	add_dependencies(test_gurobi ${catkin_EXPORTED_TARGETS})
	target_link_libraries(test_gurobi ${catkin_LIBRARIES} utils ${GUROBI_LIBRARIES} solver)

else()
	find_package(NLOPT REQUIRED)

	add_library(solver STATIC src/solver_nlopt.cpp src/bspline_utils.cpp src/nlopt_utils.cpp )
	target_compile_definitions(solver PUBLIC USE_GUROBI_FLAG=0) 
	target_include_directories (solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${DECOMP_UTIL_INCLUDE_DIRS})
	target_link_libraries(solver PUBLIC ${NLOPT_LIBRARIES} ${DECOMP_UTIL_LIBRARIES} utils octopus_search ${catkin_LIBRARIES} cgal_utils)

	add_executable(test_nlopt src/examples/test_nlopt.cpp)
	add_dependencies(test_nlopt ${catkin_EXPORTED_TARGETS})
	target_link_libraries(test_nlopt ${catkin_LIBRARIES} utils solver)

endif()


add_executable(${PROJECT_NAME}_node src/mader_node.cpp src/mader_ros.cpp src/mader.cpp )
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${catkin_LIBRARIES}  solver utils)  #${CGAL_LIBS}
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} )

add_executable(test_octopus_search src/examples/test_octopus_search.cpp src/octopus_search.cpp src/bspline_utils.cpp) 
add_dependencies(test_octopus_search ${catkin_EXPORTED_TARGETS} )
target_link_libraries(test_octopus_search ${catkin_LIBRARIES} utils cgal_utils) 

add_executable(test_utils src/examples/test_utils.cpp)
add_dependencies(test_utils ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_utils ${catkin_LIBRARIES} utils)


add_executable(test_bspline_utils src/examples/test_bspline_utils.cpp src/bspline_utils.cpp)
add_dependencies(test_bspline_utils ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_bspline_utils ${catkin_LIBRARIES})

unset(USE_GUROBI CACHE) # To avoid https://stackoverflow.com/questions/22481647/cmake-if-else-with-option


# add_library(mader STATIC src/mader.cpp src/mader.hpp)
# target_include_directories (mader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# target_link_libraries(mader PUBLIC utils ${CGAL_LIBS} solver_nlopt)


# add_library(mader_ros STATIC src/mader_ros.cpp src/mader_ros.hpp src/mader.cpp src/mader.hpp)
# target_include_directories (mader_ros PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# target_link_libraries(mader_ros PUBLIC ${catkin_LIBRARIES}  solver_nlopt utils)


# add_executable(predictor_node src/predictor_node.cpp src/predictor.cpp src/bspline_utils.cpp)
# add_dependencies(predictor_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(predictor_node ${catkin_LIBRARIES} utils)


# add_executable(test_predictor_node src/test_predictor.cpp src/predictor.cpp src/bspline_utils.cpp)
# add_dependencies(test_predictor_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_predictor_node ${catkin_LIBRARIES} utils)
