cmake_minimum_required(VERSION 2.9)
project(mtmsl_simtools C CXX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation	
	sensor_msgs
	genmsg	
	mtmsl_common	
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
	catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES minho_team
		CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
		DEPENDS sensor_msgs
	)


set(PROJECT_DIRECTORY ${PROJECT_SOURCE_DIR})

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
message(STATUS DIRS\ ${GAZEBO_LIBRARY_DIRS})
message(STATUS DIRS\ ${GAZEBO_LIBRARIES})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


##################
## Build Plugin ##
##################
set(RPLG_DIR ${PROJECT_DIRECTORY}/Plugins/MSLPlugins/robot_plugin)
include_directories(${RPLG_DIR}/include)
add_library(minho_robot SHARED ${RPLG_DIR}/src/minho_robot.cc)
set_target_properties( minho_robot
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_DIRECTORY}/bin
)
target_link_libraries(minho_robot ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(minho_robot mtmsl_common_generate_messages_cpp)

set(ORPLG_DIR ${PROJECT_DIRECTORY}/Plugins/MSLPlugins/other_plugin)
include_directories(${ORPLG_DIR}/include)
add_library(other_robot SHARED ${ORPLG_DIR}/src/other_robot.cc)
set_target_properties( other_robot
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_DIRECTORY}/bin
)
target_link_libraries(other_robot ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(other_robot mtmsl_common_generate_messages_cpp)

set(BRPLG_DIR ${PROJECT_DIRECTORY}/Plugins/MSLPlugins/bs_robot_plugin)
include_directories(${BRPLG_DIR}/include)
add_library(bs_rplugin SHARED ${BRPLG_DIR}/src/bs_rplugin.cc)
set_target_properties( bs_rplugin
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_DIRECTORY}/bin
)
target_link_libraries(bs_rplugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(bs_rplugin mtmsl_common_generate_messages_cpp)

set(WPLG_DIR ${PROJECT_DIRECTORY}/Plugins/MSLPlugins/world_plugin)
include_directories(${WPLG_DIR}/include)
add_library(minho_world SHARED ${WPLG_DIR}/src/minho_world.cc)
set_target_properties( minho_world
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_DIRECTORY}/bin
)
target_link_libraries(minho_world ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(minho_world mtmsl_common_generate_messages_cpp)

set(CA_WPLG_DIR ${PROJECT_DIRECTORY}/Plugins/CAPlugins/ca_world_plugin)
include_directories(${CA_WPLG_DIR}/include)
add_library(ca_world SHARED ${CA_WPLG_DIR}/src/ca_world.cc)
set_target_properties( ca_world
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_DIRECTORY}/bin
)
target_link_libraries(ca_world ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

set(CA_TFTPLG_DIR ${PROJECT_DIRECTORY}/Plugins/CAPlugins/ca_tft_plugin)
include_directories(${CA_TFTPLG_DIR}/include)
add_library(ca_tft SHARED ${CA_TFTPLG_DIR}/src/ca_tft.cc)
set_target_properties( ca_tft
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_DIRECTORY}/bin
)
target_link_libraries(ca_tft ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_executable(tft_test ${CA_TFTPLG_DIR}/src/main.cpp)
set_target_properties( tft_test
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_DIRECTORY}/bin
)
target_link_libraries(tft_test ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

set(CA_TRCKPLG_DIR ${PROJECT_DIRECTORY}/Plugins/CAPlugins/ca_world_plugin)
add_executable(track_test ${CA_TRCKPLG_DIR}/src/main.cpp)
set_target_properties( track_test
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_DIRECTORY}/bin
)
target_link_libraries(track_test ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

set(CA_CARTPLG_DIR ${PROJECT_DIRECTORY}/Plugins/CAPlugins/ca_cart_plugin)
include_directories(${CA_CARTPLG_DIR}/include)
add_library(ca_cart SHARED ${CA_CARTPLG_DIR}/src/ca_cart.cc)
set_target_properties( ca_cart
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_DIRECTORY}/bin
)
target_link_libraries(ca_world ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
