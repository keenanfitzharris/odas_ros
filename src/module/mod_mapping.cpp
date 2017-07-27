#include "ros/ros.h"
#include "odas_ros/hop.h"

extern "C" {
    #include "odas/odas.h"
}


void process_hops_msg(const odas_ros::hop::ConstPtr& in_ros_msg, msg_hops_obj* in_msg, msg_hops_obj* out_msg, mod_mapping_obj* mod_mapping, ros::Publisher &ros_publisher)
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


    mod_mapping_process(mod_mapping);


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
    ros::init(argc, argv, "mod_mapping");

    ROS_INFO("Initializing remapping module........");

    ros::NodeHandle node_handle("~");
    ros::NodeHandle public_handle;


    // Get parameters from server

    ROS_INFO("Retreiving parameters........");

    int fS, hopSize, nChannels;
    std::vector<int> map;
    std::string inTopic, outTopic;

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

    if(!node_handle.getParam("config/map", map)) {

        ROS_ERROR("Couln'd retrieve config/map");
        return -1;
    }

    if(!node_handle.getParam("inTopic", inTopic)) {

        ROS_ERROR("Couln'd retrieve config/inTopic");
        return -1;
    }

    if(!node_handle.getParam("outTopic", outTopic)) {

        ROS_ERROR("Couln'd retrieve config/outTopic");
        return -1;
    }


    // Contruct objects

    ROS_INFO("Constructing objects........");

    msg_hops_cfg* in_msg_cfg = msg_hops_cfg_construct();

    in_msg_cfg->fS = fS;
    in_msg_cfg->hopSize = hopSize;
    in_msg_cfg->nChannels = nChannels;

    msg_hops_obj* in_msg = msg_hops_construct(in_msg_cfg);


    msg_hops_cfg* out_msg_cfg = msg_hops_cfg_construct();

    out_msg_cfg->fS = fS;
    out_msg_cfg->hopSize = hopSize;
    out_msg_cfg->nChannels = map.size();

    msg_hops_obj* out_msg = msg_hops_construct(out_msg_cfg);


    mod_mapping_cfg* mapping_cfg = mod_mapping_cfg_construct();

    mapping_cfg->links = links_construct_zero(map.size());

    for(int iLinks = 0; iLinks < mapping_cfg->links->nLinks; iLinks++) {

        mapping_cfg->links->array[iLinks] = map.at(iLinks);
    }

    mod_mapping_obj* mod_mapping = mod_mapping_construct(mapping_cfg, out_msg_cfg);

    mod_mapping_connect(mod_mapping, in_msg, out_msg);


    // Proccess signal

    ROS_INFO("Processing signal........");

    ros::Publisher ros_publisher = public_handle.advertise<odas_ros::hop>(outTopic, 2000);
    ros::Subscriber ros_subscriber = public_handle.subscribe<odas_ros::hop>(inTopic, 2000, boost::bind(process_hops_msg, _1, in_msg, out_msg, mod_mapping, ros_publisher));
    ros::spin();


    // Destroy objects

    ROS_INFO("Destroying objects........");

    mod_mapping_disconnect(mod_mapping);

    mod_mapping_destroy(mod_mapping);
    mod_mapping_cfg_destroy(mapping_cfg);

    msg_hops_destroy(in_msg);
    msg_hops_cfg_destroy(in_msg_cfg);

    msg_hops_destroy(out_msg);
    msg_hops_cfg_destroy(out_msg_cfg);


    ROS_INFO("Quitting node........");

    return 0;
}
