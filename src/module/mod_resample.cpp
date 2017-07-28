#include "ros/ros.h"
#include "odas_ros/hop.h"

extern "C" {
    #include "odas/odas.h"
}


void process_hops_msg(const odas_ros::hop::ConstPtr& in_ros_msg, msg_hops_obj* in_msg, msg_hops_obj* out_msg, mod_resample_obj* mod_resample, ros::Publisher &ros_publisher)
{
    odas_ros::hop out_ros_msg;

    in_msg->fS = in_ros_msg->fS;
    in_msg->timeStamp = in_ros_msg->timeStamp;
    in_msg->hops->hopSize = in_ros_msg->hopSize;
    in_msg->hops->nSignals = in_ros_msg->nSignals;

    if(in_msg->fS == 0) {

        ROS_INFO("Signal ended........");

        out_ros_msg.fS = 0;
        ros_publisher.publish(out_ros_msg);

        ros::shutdown();
        return;
    }

    for (int iSample = 0; iSample < in_msg->hops->hopSize; iSample++) {

        for (int iChannel = 0; iChannel < in_msg->hops->nSignals; iChannel++) {

            in_msg->hops->array[iChannel][iSample] = in_ros_msg->data[iSample + iChannel*in_msg->hops->hopSize];
        }
    }


    mod_resample_process(mod_resample);


    out_ros_msg.fS = out_msg->fS;
    out_ros_msg.timeStamp = out_msg->timeStamp;
    out_ros_msg.hopSize = out_msg->hops->hopSize;
    out_ros_msg.nSignals = out_msg->hops->nSignals;

    out_ros_msg.data = std::vector<float>(out_ros_msg.nSignals * out_ros_msg.hopSize);

    for (int iSample = 0; iSample < out_ros_msg.hopSize; iSample++) {

        for (int iChannel = 0; iChannel < out_ros_msg.nSignals; iChannel++) {

            out_ros_msg.data[iSample + iChannel*out_ros_msg.hopSize] = out_msg->hops->array[iChannel][iSample];
        }
    }

    ros_publisher.publish(out_ros_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mod_resample");

    ROS_INFO("Initializing resample module........");

    ros::NodeHandle node_handle("~");
    ros::NodeHandle public_handle;


    // Get parameters from server

    ROS_INFO("Retreiving parameters........");

    int in_fS, in_hopSize, out_fS, out_hopSize, nChannels;
    std::string in_topic, out_topic;

    if(!node_handle.getParam("config/in/fS", in_fS)) {

        ROS_ERROR("Couln'd retrieve config/in/fS");
        return -1;
    }

    if(!node_handle.getParam("config/out/fS", out_fS)) {

        ROS_ERROR("Couln'd retrieve config/out/fS");
        return -1;
    }

    if(!node_handle.getParam("config/in/hopSize", in_hopSize)) {

        ROS_ERROR("Couln'd retrieve config/in/hopSize");
        return -1;
    }

    if(!node_handle.getParam("config/out/hopSize", out_hopSize)) {

        ROS_ERROR("Couln'd retrieve config/out/hopSize");
        return -1;
    }

    if(!node_handle.getParam("config/nChannels", nChannels)) {

        ROS_ERROR("Couln'd retrieve config/nChannels");
        return -1;
    }

    if(!node_handle.getParam("inTopic", in_topic)) {

        ROS_ERROR("Couln'd retrieve inTopic");
        return -1;
    }

    if(!node_handle.getParam("outTopic", out_topic)) {

        ROS_ERROR("Couln'd retrieve outTopic");
        return -1;
    }


    // Contruct objects

    ROS_INFO("Constructing objects........");

    msg_hops_cfg* in_msg_cfg = msg_hops_cfg_construct();

    in_msg_cfg->fS = in_fS;
    in_msg_cfg->hopSize = in_hopSize;
    in_msg_cfg->nChannels = nChannels;

    msg_hops_obj* in_msg = msg_hops_construct(in_msg_cfg);


    msg_hops_cfg* out_msg_cfg = msg_hops_cfg_construct();

    out_msg_cfg->fS = out_fS;
    out_msg_cfg->hopSize = out_hopSize;
    out_msg_cfg->nChannels = nChannels;

    msg_hops_obj* out_msg = msg_hops_construct(out_msg_cfg);


    mod_resample_cfg* resample_cfg = mod_resample_cfg_construct();

    resample_cfg->fSin = in_fS;
    resample_cfg->fSout = out_fS;

    mod_resample_obj* mod_resample = mod_resample_construct(resample_cfg, in_msg_cfg, out_msg_cfg);

    mod_resample_connect(mod_resample, in_msg, out_msg);


    // Wait for subscribers

    ROS_INFO("Waiting for subscribers........");

    ros::Publisher ros_publisher = public_handle.advertise<odas_ros::hop>(out_topic, 2000);

    while(ros_publisher.getNumSubscribers() < 1 && ros::ok()) {

        sleep(1);
    }


    // Proccess signal

    ROS_INFO("Processing signal........");

    ros::Subscriber ros_subscriber = public_handle.subscribe<odas_ros::hop>(in_topic, 2000, boost::bind(process_hops_msg, _1, in_msg, out_msg, mod_resample, ros_publisher));
    ros::spin();


    // Destroy objects

    ROS_INFO("Destroying objects........");

    mod_resample_disconnect(mod_resample);

    mod_resample_destroy(mod_resample);
    mod_resample_cfg_destroy(resample_cfg);

    msg_hops_destroy(in_msg);
    msg_hops_cfg_destroy(in_msg_cfg);

    msg_hops_destroy(out_msg);
    msg_hops_cfg_destroy(out_msg_cfg);


    ROS_INFO("Quitting node........");

    return 0;
}
