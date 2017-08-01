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


std::vector<int> retrieve_int_vector(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count) {

    std::vector<int> param_vector;

    if(!node_handle.getParam(parameter_name, param_vector)) {

        ROS_ERROR("Couln'd retrieve %s", parameter_name.c_str());
        error_count++;
    }

    return param_vector;
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


std::vector<float> retrieve_float_vector(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count)
{
    std::vector<float> param_vector;

    if(!node_handle.getParam(parameter_name, param_vector)) {

        ROS_ERROR("Couln'd retrieve %s", parameter_name.c_str());
        error_count++;
    }

    return param_vector;
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


spatialfilter_obj* retrieve_spatialfilter(ros::NodeHandle &node_handle, std::string parameter_name, int &error_count)
{
    spatialfilter_obj* spatial_filter = spatialfilter_construct_zero();

    std::vector<float> direction_vector = retrieve_float_vector(node_handle, parameter_name+"/direction", error_count);
    std::vector<float> angle_vector = retrieve_float_vector(node_handle, parameter_name+"/angle", error_count);

    if(error_count < 1) {

        for(int iDirections = 0; iDirections < 3; iDirections++)
            spatial_filter->direction[iDirections] = direction_vector[iDirections];

        spatial_filter->thetaAllPass = angle_vector[0];
        spatial_filter->thetaNoPass = angle_vector[1];
    }

    return spatial_filter;
}

char* check_filename_arg(int argc, char** argv, std::string prompt, bool check_file, int &error_count)
{
    char *string;

    if(argc != 2) {
        ROS_ERROR("Invalid arguments, please specify %s", prompt.c_str());
        error_count++;
        return nullptr;
    }

    string = argv[1];

    if(!check_file)
        return string;

    else {

        std::ifstream test_file;
        test_file.open(string);

        if(!test_file.is_open()) {

            ROS_ERROR("Can't open specified file");
            error_count++;
            return nullptr;
        }

        else {
            test_file.close();
            return string;
        }
    }
}
