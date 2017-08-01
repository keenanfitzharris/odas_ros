#include "ros/ros.h"
#include "odas_ros/track.h"
#include "../ros_parameters_parser.h"

extern "C" {
    #include "odas/odas.h"
}


void process_tracks_msg(const odas_ros::track::ConstPtr &ros_tracks_msg, msg_tracks_obj* tracks_msg, snk_tracks_obj* snk_tracks)
{
    tracks_msg->timeStamp = ros_tracks_msg->timeStamp;
    tracks_msg->fS = ros_tracks_msg->fS;
    tracks_msg->tracks->nTracks = ros_tracks_msg->nTracks;

    if(tracks_msg->fS == 0) {
        ROS_INFO("Signal ended........");

        ros::shutdown();
        return;
    }

    int nTracks = tracks_msg->tracks->nTracks;

    for(int iTracks = 0; iTracks < nTracks; iTracks++) {

        tracks_msg->tracks->ids[iTracks] = ros_tracks_msg->ids[iTracks];

        for(int iSamples = 0; iSamples < 3; iSamples++)
            tracks_msg->tracks->array[iSamples + iTracks * 3] = ros_tracks_msg->data[iSamples + iTracks * 3];
    }

    snk_tracks_process(snk_tracks);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "snk_tracks_file", ros::init_options::AnonymousName);

    ROS_INFO("Initializing sink tracks file........");

    ros::NodeHandle node_handle("~");
    ros::NodeHandle public_handle;


    // Get filename from arguments

    ROS_INFO("Reading filename........");

    int error_count = 0;

    char* filename = check_filename_arg(argc, argv, "filename", false, error_count);


    // Get parameters from server

    ROS_INFO("Retrieving parameters........");

    int fS = retrieve_int(node_handle, "config/fS", error_count);
    int nTracks = retrieve_int(node_handle, "config/nTracks", error_count);

    std::string inTopic = retrieve_string(node_handle, "inTopic", error_count);

    if(error_count > 0) {
        ROS_ERROR("Failed to retrieve parameters");
        return -1;
    }


    // Contruct objects

    ROS_INFO("Constructing objects........");

    msg_tracks_cfg* tracks_cfg = msg_tracks_cfg_construct();

    tracks_cfg->fS = fS;
    tracks_cfg->nTracks = nTracks;

    msg_tracks_obj* tracks_msg = msg_tracks_construct(tracks_cfg);


    snk_tracks_cfg* snk_cfg = snk_tracks_cfg_construct();

    snk_cfg->format = format_construct_float();
    snk_cfg->interface = interface_construct_file(filename);
    snk_cfg->fS = fS;

    snk_tracks_obj* snk_tracks = snk_tracks_construct(snk_cfg, tracks_cfg);

    snk_tracks_connect(snk_tracks, tracks_msg);
    snk_tracks_open(snk_tracks);


    // Proccess signal

    ROS_INFO("Processing signal........");

    ros::Subscriber ros_subscriber = public_handle.subscribe<odas_ros::track>(inTopic, 2000, boost::bind(process_tracks_msg, _1, tracks_msg, snk_tracks));
    ros::spin();


    // Destroy objects

    ROS_INFO("Destroying objects........");

    snk_tracks_close(snk_tracks);
    snk_tracks_disconnect(snk_tracks);

    snk_tracks_destroy(snk_tracks);
    snk_tracks_cfg_destroy(snk_cfg);

    msg_tracks_destroy(tracks_msg);
    msg_tracks_cfg_destroy(tracks_cfg);


    ROS_INFO("Quitting node........");

    return 0;
}
