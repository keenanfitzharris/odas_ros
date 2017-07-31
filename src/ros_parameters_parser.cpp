#include "ros_parameters_parser.h"

int retrieve_int(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count)
{
    int param_value;

    if(!node_handle.getParam(parameter_name, param_value)) {

        ROS_ERROR("Couln'd retrieve %s", parameter_name.c_str());
        error_count++;
    }

    return param_value;
}


float retrieve_float(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count)
{
    float param_value;

    if(!node_handle.getParam(parameter_name, param_value)) {

        ROS_ERROR("Couln'd retrieve %s", parameter_name.c_str());
        error_count++;
    }

    return param_value;
}


float retrieve_string_float(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count)
{
    std::string epsilon_str;
    float epsilon;

    if(!node_handle.getParam(parameter_name, epsilon_str)) {

        ROS_ERROR("Couln'd retrieve %s", parameter_name.c_str());
        error_count++;
    }

    try {

        epsilon = atof(epsilon_str.c_str());

    } catch(...) {

        ROS_ERROR("Error while parsing %s", parameter_name.c_str());
        error_count++;
    }

    return epsilon;
}


std::string retrieve_string(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count)
{
    std::string param_value;

    if(!node_handle.getParam(parameter_name, param_value)) {

        ROS_ERROR("Couln'd retrieve %s", parameter_name.c_str());
        error_count++;
    }

    return param_value;
}


mics_obj* retrieve_mics(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count)
{
    XmlRpc::XmlRpcValue mics_param, mic_buffer;
    mics_obj* mics;
    int nChannels;

    if(!node_handle.getParam(parameter_name, mics_param)) {

        ROS_ERROR("Couln'd retrieve %s", parameter_name.c_str());
        error_count++;
        return mics_construct_zero(0);
    }

    nChannels = mics_param.size();
    mics = mics_construct_zero(nChannels);

    try {

        for(int iChannels = 0; iChannels < nChannels; iChannels++) {

            mic_buffer = mics_param[iChannels];

            for(int iMu = 0; iMu < 3; iMu++)
                mics->mu[iMu + 3*iChannels] = (float)static_cast<double>(mic_buffer["mu"][iMu]);

            for(int iSigma2 = 0; iSigma2 < 9; iSigma2++)
                mics->sigma2[iSigma2 + 9*iChannels] = (float)static_cast<double>(mic_buffer["sigma2"][iSigma2]);

            for(int iDirection = 0; iDirection < 3; iDirection++)
                mics->direction[iDirection + 3*iChannels] = (float)static_cast<double>(mic_buffer["direction"][iDirection]);

            mics->thetaAllPass[iChannels] = (float)static_cast<double>(mic_buffer["angle"][0]);
            mics->thetaNoPass[iChannels] = (float)static_cast<double>(mic_buffer["angle"][1]);
        }

    } catch(...) {

        ROS_ERROR("Error while parsing %s", parameter_name.c_str());
        error_count++;
        return mics_construct_zero(0);
    }

    return mics;
}
