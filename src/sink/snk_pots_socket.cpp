#include "ros/ros.h"
#include "odas_ros/pot.h"
#include "../ros_parameters_parser.h"

extern "C" {
    #include "odas/odas.h"
}


void process_pots_msg(const odas_ros::pot::ConstPtr &ros_pots_msg, msg_pots_obj* pots_msg, snk_pots_obj* snk_pots)
{
    pots_msg->timeStamp = ros_pots_msg->timeStamp;
    pots_msg->fS = ros_pots_msg->fS;
    pots_msg->pots->nPots = ros_pots_msg->nPots;

    if(pots_msg->fS == 0) {
        ROS_INFO("Signal ended........");

        ros::shutdown();
        return;
    }

    int nSamples = pots_msg->pots->nPots * 4;

    for(int iSamples = 0; iSamples < nSamples; iSamples++)
        pots_msg->pots->array[iSamples] = ros_pots_msg->data[iSamples];

    snk_pots_process(snk_pots);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "snk_pots_file", ros::init_options::AnonymousName);

    ROS_INFO("Initializing sink pots file........");

    ros::NodeHandle node_handle("~");
    ros::NodeHandle public_handle;


    // Get parameters from server

    ROS_INFO("Retrieving parameters........");

    int error_count = 0;

    int fS = retrieve_int(node_handle, "config/fS", error_count);
    int nPots = retrieve_int(node_handle, "config/nPots", error_count);

    std::string ip = retrieve_string(node_handle, "config/ip", error_count);
    int port = retrieve_int(node_handle, "config/port", error_count);

    std::string inTopic = retrieve_string(node_handle, "inTopic", error_count);

    if(error_count > 0) {
        ROS_ERROR("Failed to retrieve parameters");
        return -1;
    }


    // Contruct objects

    ROS_INFO("Constructing objects........");

    msg_pots_cfg* pots_cfg = msg_pots_cfg_construct();

    pots_cfg->fS = fS;
    pots_cfg->nPots = nPots;

    msg_pots_obj* msg_pots = msg_pots_construct(pots_cfg);

    snk_pots_cfg* snk_cfg = snk_pots_cfg_construct();

    snk_cfg->format = format_construct_float();
    snk_cfg->interface = interface_construct_socket(ip.c_str(),port);
    snk_cfg->fS = fS;

    snk_pots_obj* snk_pots = snk_pots_construct(snk_cfg, pots_cfg);

    snk_pots_connect(snk_pots, msg_pots);
    snk_pots_open(snk_pots);


    // Proccess signal

    ROS_INFO("Processing signal........");

    ros::Subscriber ros_subscriber = public_handle.subscribe<odas_ros::pot>(inTopic, 2000, boost::bind(process_pots_msg, _1, msg_pots, snk_pots));
    ros::spin();


    // Destroy objects

    ROS_INFO("Destroying objects........");

    snk_pots_close(snk_pots);
    snk_pots_disconnect(snk_pots);

    snk_pots_destroy(snk_pots);
    snk_pots_cfg_destroy(snk_cfg);

    msg_pots_destroy(msg_pots);
    msg_pots_cfg_destroy(pots_cfg);


    ROS_INFO("Quitting node........");

    return 0;
}
