cmake_minimum_required(VERSION 2.8.3)
project(odas_ros)

# Root folder of ODAS library
set(ODAS_LOCATION $ENV{HOME}/Documents/odas)

find_package(
	catkin REQUIRED COMPONENTS
	roscpp message_generation
)

 add_message_files(
	FILES
	hop.msg
	spectra.msg
 )

 generate_messages()



catkin_package(
	CATKIN_DEPENDS
	roscpp
	message_runtime
)

include_directories(
 	${catkin_INCLUDE_DIRS}
	${ODAS_LOCATION}/include
)


#
# Build sources
#

# Raw hops from file
add_executable(
	src_hops_file
	src/source/src_hops_file.cpp
)

add_dependencies(
	src_hops_file
	mars_ros_generate_messages_cpp
)

target_link_libraries(
	src_hops_file
	${catkin_LIBRARIES}
	${ODAS_LOCATION}/lib/libodas.so
)


#
# Build modules
#

# Mapping module
add_executable(
	mod_mapping
	src/module/mod_mapping.cpp
)

add_dependencies(
	mod_mapping
	mars_ros_generate_messages_cpp
)

target_link_libraries(
	mod_mapping
	${catkin_LIBRARIES}
	${ODAS_LOCATION}/lib/libodas.so
)


#
# Build sinks
#

# Raw hops to file
add_executable(
	snk_hops_file
	src/sink/snk_hops_file.cpp
)

add_dependencies(
	snk_hops_file
	mars_ros_generate_messages_cpp
)

target_link_libraries(
	snk_hops_file
	${catkin_LIBRARIES}
	${ODAS_LOCATION}/lib/libodas.so
)
