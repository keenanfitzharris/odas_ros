#ifndef ros_parameter_parser_h
#define ros_parameter_parser_h

#include "ros/ros.h"

#include <fstream>

extern "C" {
    #include "odas/odas.h"
}

int retrieve_int(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);
std::vector<int> retrieve_int_vector(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);

float retrieve_float(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);
std::vector<float> retrieve_float_vector(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);
float retrieve_string_float(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);

std::string retrieve_string(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);

mics_obj* retrieve_mics(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);
spatialfilter_obj* retrieve_spatialfilter(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);

samplerate_obj* retrieve_samplerate(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);
soundspeed_obj* retrieve_soundspeed(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);

char* check_filename_arg(int argc, char** argv, std::string prompt, bool check_file, int &error_count);

#endif
