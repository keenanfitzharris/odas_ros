#include "ros/ros.h"
#include "odas_ros/spectra.h"
#include "odas_ros/track.h"

#include "../ros_parameters_parser.h"
#include "../msg_synchroniser.h"

extern "C" {
    #include "odas/odas.h"
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mod_sss");

    ROS_INFO("Initializing sspf module........");

    ros::NodeHandle node_handle("~");
    ros::NodeHandle public_handle;


    // Get parameters from server

    ROS_INFO("Retreiving parameters........");

    int error_count = 0;

    int fS = retrieve_int(node_handle, "general/fS", error_count);
    int halfFrameSize = retrieve_int(node_handle, "general/frameSize", error_count) / 2;
    mics_obj* mics = retrieve_mics(node_handle, "general/mics", error_count);
    spatialfilter_obj* spatial_filter = retrieve_spatialfilter(node_handle, "general/spatialfilter", error_count);
    float epsilon = retrieve_string_float(node_handle, "general/epsilon", error_count);

    int nTracks = retrieve_int(node_handle, "config/nTracks", error_count);
    int nThetas = retrieve_int(node_handle, "config/nThetas", error_count);
    float gainMin = retrieve_float(node_handle, "config/gainMin", error_count);

    std::string stft_topic = retrieve_string(node_handle, "stftTopic", error_count);
    std::string sss_topic = retrieve_string(node_handle, "sssTopic", error_count);
    std::string tracks_topic = retrieve_string(node_handle, "tracksTopic", error_count);
    std::string sspf_topic = retrieve_string(node_handle, "outTopic", error_count);

    if(error_count != 0) {
        ROS_ERROR("Failed to retrieved parameters");
        return -1;
    }


    // Contruct objects

    ROS_INFO("Constructing objects........");

    sspf_synchroniser_obj msg_synchroniser;

    msg_synchroniser.stft_msg_cfg->fS =fS;
    msg_synchroniser.stft_msg_cfg->halfFrameSize = halfFrameSize;
    msg_synchroniser.stft_msg_cfg->nChannels = mics->nChannels;

    msg_synchroniser.sss_msg_cfg->fS = fS;
    msg_synchroniser.sss_msg_cfg->halfFrameSize = halfFrameSize;
    msg_synchroniser.sss_msg_cfg->nChannels = nTracks;

    msg_synchroniser.tracks_msg_cfg->fS = fS;
    msg_synchroniser.tracks_msg_cfg->nTracks = nTracks;

    msg_synchroniser.sspf_cfg->epsilon = epsilon;
    msg_synchroniser.sspf_cfg->gainMin = gainMin;
    msg_synchroniser.sspf_cfg->mics = mics;
    msg_synchroniser.sspf_cfg->spatialfilter = spatial_filter;
    msg_synchroniser.sspf_cfg->nThetas = nThetas;

    msg_synchroniser.sspf_msg_cfg->fS = fS;
    msg_synchroniser.sspf_msg_cfg->halfFrameSize = halfFrameSize;
    msg_synchroniser.sspf_msg_cfg->nChannels = nTracks;

    msg_synchroniser.construct_module();


    // Wait for subscribers

    ROS_INFO("Waiting for subscribers........");

    ros::Publisher ros_publiser = public_handle.advertise<odas_ros::spectra>(sspf_topic, 2000);
    msg_synchroniser.ros_publisher = &ros_publiser;

    while(msg_synchroniser.ros_publisher->getNumSubscribers() < 1 && ros::ok()) {

        sleep(1);
    }

    // Proccess signal

    ROS_INFO("Processing signal........");

    ros::Subscriber sspf_subscriber = public_handle.subscribe<odas_ros::spectra>(stft_topic, 2000, &sspf_synchroniser_obj::process_stft_msg, &msg_synchroniser);
    ros::Subscriber sss_subscriber = public_handle.subscribe<odas_ros::spectra>(sss_topic, 2000, &sspf_synchroniser_obj::process_sss_msg, &msg_synchroniser);
    ros::Subscriber tracks_subscriber = public_handle.subscribe<odas_ros::track>(tracks_topic, 2000, &sspf_synchroniser_obj::process_tracks_msg, &msg_synchroniser);

    ros::spin();


    ROS_INFO("Quitting node........");

    return 0;
}
