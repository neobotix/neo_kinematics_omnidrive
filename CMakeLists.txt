cmake_minimum_required(VERSION 2.8.3)
project(neo_kinematics_omnidrive)

set(CMAKE_CXX_STANDARD 11)

## By default build with code optimization and no debug info
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release"
  	  CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            dynamic_reconfigure
            roscpp
            tf
            neo_srvs
            neo_msgs
            neo_common
        )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## dynamic reconfigure
#generate_dynamic_reconfigure_options(
#    cfg/NeoPlanner.cfg
#)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        dynamic_reconfigure
        roscpp
		tf
        neo_srvs
		neo_msgs
		neo_common
)

add_executable(neo_omnidrive_node src/neo_omnidrive_node.cpp)
add_dependencies(neo_omnidrive_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(neo_omnidrive_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_LIBRARIES})

add_executable(neo_omnidrive_simulation_node src/neo_omnidrive_simulation_node.cpp)
add_dependencies(neo_omnidrive_simulation_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(neo_omnidrive_simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_LIBRARIES})

add_executable(neo_omnidrive_socketcan src/neo_omnidrive_socketcan.cpp)
add_dependencies(neo_omnidrive_socketcan ${catkin_EXPORTED_TARGETS})
target_link_libraries(neo_omnidrive_socketcan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_LIBRARIES})

add_executable(test_velocity_solver test/test_velocity_solver.cpp)
add_executable(test_omni_kinematics test/test_omni_kinematics.cpp)

install(TARGETS neo_omnidrive_node neo_omnidrive_socketcan
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
