#include "ros/ros.h"
#include "odas_ros/spectra.h"
#include "../ros_parameters_parser.h"

extern "C" {
    #include "odas/odas.h"
}


void process_spectra_msg(const odas_ros::spectra::ConstPtr spectra_ros_msg, msg_spectra_obj* spectra_msg, snk_spectra_obj* snk_spectra)
{
    spectra_msg->timeStamp = spectra_ros_msg->timeStamp;
    spectra_msg->fS = spectra_ros_msg->fS;
    spectra_msg->freqs->nSignals = spectra_ros_msg->nSignals;
    spectra_msg->freqs->halfFrameSize = spectra_ros_msg->halfFrameSize;

    if(spectra_msg->fS == 0) {

        ROS_INFO("Signal ended........");
        ros::shutdown();
        return;
    }

    int nSignals = spectra_msg->freqs->nSignals;
    int nSamples = spectra_msg->freqs->halfFrameSize * 2;

    for(int iSignals = 0; iSignals < nSignals; iSignals++)
        for(int iSamples = 0; iSamples < nSamples; iSamples++)
            spectra_msg->freqs->array[iSignals][iSamples] = spectra_ros_msg->data[iSamples + iSignals * nSamples];

    snk_spectra_process(snk_spectra);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "snk_spectra_file", ros::init_options::AnonymousName);

    ROS_INFO("Initializing sink spectra file........");

    ros::NodeHandle node_handle("~");
    ros::NodeHandle public_handle;


    // Get filename from arguments

    ROS_INFO("Reading filename........");

    int error_count = 0;

    char* filename = check_filename_arg(argc, argv, "filename", false, error_count);


    // Get parameters from server

    ROS_INFO("Retrieving parameters........");

    int fS = retrieve_int(node_handle, "config/fS", error_count);
    int halfFrameSize = retrieve_int(node_handle, "config/frameSize", error_count) / 2;
    int nChannels = retrieve_int(node_handle, "config/nChannels", error_count);

    std::string inTopic = retrieve_string(node_handle, "inTopic", error_count);

    if(error_count > 0) {
        ROS_ERROR("Failed to retrieve parameters");
        return -1;
    }


    // Contruct objects

    ROS_INFO("Constructing objects........");

    msg_spectra_cfg* spectra_cfg = msg_spectra_cfg_construct();

    spectra_cfg->fS = fS;
    spectra_cfg->halfFrameSize = halfFrameSize;
    spectra_cfg->nChannels = nChannels;

    msg_spectra_obj* spectra_msg = msg_spectra_construct(spectra_cfg);


    snk_spectra_cfg* snk_cfg = snk_spectra_cfg_construct();

    snk_cfg->format = format_construct_float();
    snk_cfg->interface = interface_construct_file(filename);
    snk_cfg->fS = fS;

    snk_spectra_obj* snk_spectra = snk_spectra_construct(snk_cfg, spectra_cfg);


    snk_spectra_open(snk_spectra);
    snk_spectra_connect(snk_spectra, spectra_msg);


    // Proccess signal

    ROS_INFO("Processing signal........");

    ros::Subscriber ros_subscriber = public_handle.subscribe<odas_ros::spectra>(inTopic, 2000, boost::bind(process_spectra_msg, _1, spectra_msg, snk_spectra));
    ros::spin();


    // Destroy objects

    ROS_INFO("Destroying objects........");

    snk_spectra_disconnect(snk_spectra);
    snk_spectra_close(snk_spectra);

    snk_spectra_destroy(snk_spectra);
    snk_spectra_cfg_destroy(snk_cfg);

    msg_spectra_destroy(spectra_msg);
    msg_spectra_cfg_destroy(spectra_cfg);


    ROS_INFO("Quitting node........");

    return 0;
}
