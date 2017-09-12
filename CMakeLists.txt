cmake_minimum_required(VERSION 3.0.0)
project(odas_ros)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(
	catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation

)

#External ODAS git repository
#TODO change revision number to latest suitable version
include(ExternalProject)
ExternalProject_Add(odas_lib
	GIT_REPOSITORY https://github.com/introlab/odas.git
	GIT_TAG 85f27f082330162f80b584ece28fabbdb5012a62
#	GIT_TAG dev
	BUILD_IN_SOURCE
	SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/odas_git
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:String=${CATKIN_DEVEL_PREFIX}
)

add_message_files(
	FILES
	hop.msg
	spectra.msg
	pot.msg
        track.msg
 )

generate_messages()

catkin_package(
	CATKIN_DEPENDS
	roscpp
        std_msgs
	message_runtime
	LIBRARIES odas_lib
)

# Root folder of ODAS library
set(ODAS_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/odas_git)
set(ODAS_LIBRARY "-L${CATKIN_DEVEL_PREFIX}/lib -lodas")


include_directories(
 	${catkin_INCLUDE_DIRS}
	${ODAS_LOCATION}/include
)


#
# Build sources
#

# Hops from file
add_executable(src_hops_file 
	src/source/src_hops_file.cpp
)

add_dependencies(src_hops_file 
	odas_ros_generate_messages_cpp 
	odas_lib
)

target_link_libraries(
	src_hops_file
	${catkin_LIBRARIES}
	${ODAS_LIBRARY}
)

# Hops from soundcard
add_executable(
        src_hops_soundcard
        src/source/src_hops_soundcard.cpp
)

add_dependencies(
        src_hops_soundcard
	odas_ros_generate_messages_cpp 
	odas_lib
)

target_link_libraries(
        src_hops_soundcard
	${catkin_LIBRARIES}
	${ODAS_LIBRARY}
)


# Spectra from file
#add_executable(
#        src_spectra_file
#        src/source/src_spectra_file.cpp
#        src/ros_parameters_parser.cpp
#)
#
#add_dependencies(
#        src_spectra_file
#       mars_ros_generate_messages_cpp
#)
#
#target_link_libraries(
#        src_spectra_file
#        ${catkin_LIBRARIES}
#        ${ODAS_LIBRARY}
#)


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
	odas_ros_generate_messages_cpp 
	odas_lib
)

target_link_libraries(
	mod_mapping
	${catkin_LIBRARIES}
	${ODAS_LIBRARY}
)


# Resample module
add_executable(
	mod_resample
	src/module/mod_resample.cpp
)

add_dependencies(
	mod_resample
	odas_ros_generate_messages_cpp 
	odas_lib
)

target_link_libraries(
	mod_resample
	${catkin_LIBRARIES}
	${ODAS_LIBRARY}
)


# STFT module
add_executable(
	mod_stft
	src/module/mod_stft.cpp
)

add_dependencies(
	mod_stft
	odas_ros_generate_messages_cpp 
	odas_lib
)

target_link_libraries(
	mod_stft
	${catkin_LIBRARIES}
	${ODAS_LIBRARY}
)


# SSL module
add_executable(
	mod_ssl
	src/module/mod_ssl.cpp
        src/ros_parameters_parser.cpp
)

add_dependencies(
	mod_ssl
	odas_ros_generate_messages_cpp 
	odas_lib
)

target_link_libraries(
	mod_ssl
	${catkin_LIBRARIES}
	${ODAS_LIBRARY}
)


# SST module
add_executable(
	mod_sst
	src/module/mod_sst.cpp
        src/ros_parameters_parser.cpp
)

add_dependencies(
	mod_sst
	odas_ros_generate_messages_cpp 
	odas_lib
)

target_link_libraries(
	mod_sst
	${catkin_LIBRARIES}
	${ODAS_LIBRARY}
)


# SSS module (disabled for now because of API change)
#add_executable(
#        mod_sss
#        src/module/mod_sss.cpp
#        src/ros_parameters_parser.cpp
#)

#add_dependencies(
#        mod_sss
#        odas_ros_generate_messages_cpp
#)

#target_link_libraries(
#        mod_sss
#        ${catkin_LIBRARIES}
#        ${ODAS_LIBRARY}
#)


# SSPF module  (disabled for now because of API change)
#add_executable(
#        mod_sspf
#        src/module/mod_sspf.cpp
#        src/ros_parameters_parser.cpp
#        src/msg_synchroniser.cpp
#)

#add_dependencies(
#        mod_sspf
#        odas_ros_generate_messages_cpp 
#	 odas_lib
#)

#target_link_libraries(
#        mod_sspf
#        ${catkin_LIBRARIES}
#        ${ODAS_LIBRARY}
#)


#
# Build sinks
#

# Hops to file
add_executable(
	snk_hops_file
	src/sink/snk_hops_file.cpp
)

add_dependencies(
	snk_hops_file
	odas_ros_generate_messages_cpp 
	odas_lib
)

target_link_libraries(
	snk_hops_file
	${catkin_LIBRARIES}
	${ODAS_LIBRARY}
)


# Spectra to file
add_executable(
        snk_spectra_file
        src/sink/snk_spectra_file.cpp
        src/ros_parameters_parser.cpp
)

add_dependencies(
        snk_spectra_file
        odas_ros_generate_messages_cpp 
	odas_lib
)

target_link_libraries(
        snk_spectra_file
        ${catkin_LIBRARIES}
        ${ODAS_LIBRARY}
)


# Pots to file
add_executable(
        snk_pots_file
        src/sink/snk_pots_file.cpp
        src/ros_parameters_parser.cpp
)

add_dependencies(
        snk_pots_file
        odas_ros_generate_messages_cpp 
	odas_lib
)

target_link_libraries(
        snk_pots_file
        ${catkin_LIBRARIES}
        ${ODAS_LIBRARY}
)


# Pots to socket
add_executable(
        snk_pots_socket
        src/sink/snk_pots_socket.cpp
        src/ros_parameters_parser.cpp
)

add_dependencies(
        snk_pots_socket
        odas_ros_generate_messages_cpp 
	odas_lib
)

target_link_libraries(
        snk_pots_socket
        ${catkin_LIBRARIES}
        ${ODAS_LIBRARY}
)


# Pots to renard
add_executable(
        snk_pots_renard
        src/sink/snk_pots_renard.cpp
        src/ros_parameters_parser.cpp
)

add_dependencies(
        snk_pots_renard
        odas_ros_generate_messages_cpp 
	odas_lib
)

target_link_libraries(
        snk_pots_renard
        ${catkin_LIBRARIES}
        ${ODAS_LIBRARY}
)


# Tracks to file
add_executable(
        snk_tracks_file
        src/sink/snk_tracks_file.cpp
        src/ros_parameters_parser.cpp
)

add_dependencies(
        snk_tracks_file
        odas_ros_generate_messages_cpp 
	odas_lib
)

target_link_libraries(
        snk_tracks_file
        ${catkin_LIBRARIES}
        ${ODAS_LIBRARY}
)
