#ifndef ros_parameter_parser_h
#define ros_parameter_parser_h

#include "ros/ros.h"

extern "C" {
    #include "odas/odas.h"
}

int retrieve_int(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);

float retrieve_float(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);
float retrieve_string_float(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);

std::string retrieve_string(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);

mics_obj* retrieve_mics(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count);

#endif
