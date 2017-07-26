#include "ros/ros.h"
#include "odas_ros/hop.h"

extern "C" {
	#include "odas/odas.h"
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "src_hops_file");

    ROS_INFO("Initializing source hops file........");

    ros::NodeHandle node_handle("~");
    ros::NodeHandle public_handle;


    // Get filame from arguments

    ROS_INFO("Reading filename........");

    if(argc < 2) {

        ROS_ERROR("Please provide source filename");
        return -1;
    }

    else if(argc > 2) {

        ROS_ERROR("Too many arguments, expecting only source filename");
        return -1;
    }

    char *filename = argv[1];


    // Get parameters from server

    ROS_INFO("Retreiving parameters........");

    int fS, hopSize, nChannels, nBits;
    std::string outTopic;

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

    if(!node_handle.getParam("outTopic", outTopic)) {

        ROS_ERROR("Couln'd retrieve outTopic");
        return -1;
    }

    ros::Publisher ros_publisher = public_handle.advertise<odas_ros::hop>(outTopic, 2000);


    // Contruct objects

    ROS_INFO("Constructing objects........");

    src_hops_cfg* hops_cfg = src_hops_cfg_construct();

    hops_cfg->format = format_construct_bin(nBits);
    hops_cfg->interface = interface_construct_file(filename);
    hops_cfg->fS = fS;


    msg_hops_cfg* msg_cfg = msg_hops_cfg_construct();

    msg_cfg->fS = fS;
    msg_cfg->hopSize = hopSize;
    msg_cfg->nChannels = nChannels;


    msg_hops_obj* msg_out = msg_hops_construct(msg_cfg);


    src_hops_obj* src_obj = src_hops_construct(hops_cfg, msg_cfg);

    src_hops_connect(src_obj, msg_out);
    src_hops_open(src_obj);


    // Wait for subscribers

    ROS_INFO("Waiting for subscribers........");

    while(ros_publisher.getNumSubscribers() < 1) {

        sleep(1);
    }


    // Proccess signal

    ROS_INFO("Processing signal........");

    int return_value = 0;

    ros::Rate playing_rate(fS/hopSize);

    while(return_value == 0 && ros::ok()) {

        return_value = src_hops_process(src_obj);

        odas_ros::hop ros_msg_out;

        ros_msg_out.timeStamp = msg_out->timeStamp;
        ros_msg_out.nSignals = msg_out->hops->nSignals;
        ros_msg_out.hopSize = msg_out->hops->hopSize;
        ros_msg_out.fS = msg_out->fS;

        ros_msg_out.data = std::vector<float>(ros_msg_out.nSignals * ros_msg_out.hopSize);

        for (int iSample = 0; iSample < ros_msg_out.hopSize; iSample++) {

            for (int iChannel = 0; iChannel < ros_msg_out.nSignals; iChannel++) {

                ros_msg_out.data[iSample + iChannel*ros_msg_out.hopSize] = msg_out->hops->array[iChannel][iSample];
            }
        }

        ros_publisher.publish(ros_msg_out);

        playing_rate.sleep();
    }

    src_hops_disconnect(src_obj);
    src_hops_close(src_obj);


    // Destroy objects

    ROS_INFO("Destroying objects........");

    src_hops_destroy(src_obj);
    msg_hops_destroy(msg_out);

    msg_hops_cfg_destroy(msg_cfg);
    src_hops_cfg_destroy(hops_cfg);

    ROS_INFO("Quitting node........");

    return 0;
}
