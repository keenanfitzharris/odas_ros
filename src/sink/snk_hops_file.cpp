#include "ros/ros.h"
#include "odas_ros/hop.h"
#include <fstream>

extern "C" {
    #include "odas/odas.h"
}


void process_hops_msg(const odas_ros::hop::ConstPtr& ros_msg, msg_hops_obj* in_msg, snk_hops_obj* snk_hops)
{
    in_msg->fS = ros_msg->fS;
    in_msg->timeStamp = ros_msg->timeStamp;
    in_msg->hops->hopSize = ros_msg->hopSize;
    in_msg->hops->nSignals = ros_msg->nSignals;

    if(in_msg->fS == 0) {

        ROS_INFO("Signal ended........");
        ros::shutdown();
        return;
    }

    for (int iSample = 0; iSample < in_msg->hops->hopSize; iSample++) {

        for (int iChannel = 0; iChannel < in_msg->hops->nSignals; iChannel++) {

            in_msg->hops->array[iChannel][iSample] = ros_msg->data[iSample + iChannel*in_msg->hops->hopSize];
        }
    }

    snk_hops_process(snk_hops);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "snk_hops_file", ros::init_options::AnonymousName);

    ROS_INFO("Initializing sink hops file........");

    ros::NodeHandle node_handle("~");
    ros::NodeHandle public_handle;


    // Get filename from arguments

    ROS_INFO("Reading filename........");

    if(argc < 2) {

        ROS_ERROR("Please provide sink filename");
        return -1;
    }

    else if(argc > 4) {

        ROS_ERROR("Too many arguments, expecting only filename");
        return -1;
    }

    char *filename = argv[1];


    std::ofstream test_file;
    test_file.open(filename);

    if(!test_file.is_open()) {

        ROS_ERROR("Can't open specified output file");
        return -1;
    }

    else {
        test_file.close();
    }


    // Get parameters from server

    ROS_INFO("Retreiving parameters........");

    int fS, hopSize, nChannels, nBits;
    std::string inTopic;

    if(!node_handle.getParam("config/fS", fS)) {

        ROS_ERROR("Couln'd retrieve config/fS");
        return -1;
    }

    if(!node_handle.getParam("config/hopSize", hopSize)) {

        ROS_ERROR("Couln'd retrieve config/hopSize");
        return -1;
    }

    if(!node_handle.getParam("config/nChannels", nChannels)) {

        ROS_ERROR("Couln'd retrieve config/nChannels");
        return -1;
    }

    if(!node_handle.getParam("config/nBits", nBits)) {

        ROS_ERROR("Couln'd retrieve config/nBits");
        return -1;
    }

    if(!node_handle.getParam("inTopic", inTopic)) {

        ROS_ERROR("Couln'd retrieve config/inTopic");
        return -1;
    }


    // Contruct objects

    ROS_INFO("Constructing objects........");

    msg_hops_cfg* msg_cfg = msg_hops_cfg_construct();

    msg_cfg->fS = fS;
    msg_cfg->hopSize = hopSize;
    msg_cfg->nChannels = nChannels;


    snk_hops_cfg* snk_cfg = snk_hops_cfg_construct();

    snk_cfg->fS = fS;
    snk_cfg->format = format_construct_bin(nBits);
    snk_cfg->interface = interface_construct_file(filename);


    msg_hops_obj* in_msg = msg_hops_construct(msg_cfg);


    snk_hops_obj* snk_hops = snk_hops_construct(snk_cfg, msg_cfg);

    snk_hops_connect(snk_hops, in_msg);
    snk_hops_open(snk_hops);



    // Proccess signal

    ROS_INFO("Processing signal........");

    ros::Subscriber ros_subscriber = public_handle.subscribe<odas_ros::hop>(inTopic, 2000, boost::bind(process_hops_msg, _1, in_msg, snk_hops));
    ros::spin();


    // Destroy objects

    ROS_INFO("Destroying objects........");

    snk_hops_disconnect(snk_hops);
    snk_hops_close(snk_hops);
    snk_hops_destroy(snk_hops);

    msg_hops_destroy(in_msg);

    snk_hops_cfg_destroy(snk_cfg);
    msg_hops_cfg_destroy(msg_cfg);


    ROS_INFO("Quitting node........");

    return 0;
}
